#include "OctNavVolume3D.h"
#include "NavNode.h"
#include "ProceduralMeshComponent.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Containers/Queue.h" // TQueue

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <utility>

#include "Kismet/KismetMathLibrary.h"
#include "GameFramework/Actor.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"

/// Global material reference for the debug grid
static UMaterial* GridMaterial = nullptr;

//
// ============================================================================
// AOctNavVolume3D – 3D Grid Navigation Volume with Octree-Based Occlusion
// ============================================================================
// - Builds a 3D navigation grid composed of NavNodes
// - Visualizes the grid with a procedural mesh (debug grid)
// - Builds an octree for coarse collision / blockage tests
// - Provides A* pathfinding over the grid
// - Supports finding nearest free node via BFS with collision checks
// ============================================================================
//

AOctNavVolume3D::AOctNavVolume3D()
{
	PrimaryActorTick.bCanEverTick = true;

	// -------------------------------------------------------
	// Scene / Root Setup
	// -------------------------------------------------------
	DefaultRootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultRootComponent"));
	SetRootComponent(DefaultRootComponent);

	// -------------------------------------------------------
	// Procedural Mesh Setup (visual debug grid)
	// -------------------------------------------------------
	ProceduralMeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh"));
	ProceduralMeshComponent->SetupAttachment(DefaultRootComponent);
	ProceduralMeshComponent->SetCastShadow(false);
	ProceduralMeshComponent->SetEnableGravity(false);
	ProceduralMeshComponent->bApplyImpulseOnDamage = false;
	ProceduralMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ProceduralMeshComponent->SetGenerateOverlapEvents(false);
	ProceduralMeshComponent->CanCharacterStepUpOn = ECanBeCharacterBase::ECB_No;

	// Grid is visible in-game unless the actor itself is hidden.
	ProceduralMeshComponent->SetHiddenInGame(false);

	// -------------------------------------------------------
	// Load debug grid material from plugin content
	// -------------------------------------------------------
	static ConstructorHelpers::FObjectFinder<UMaterial> MaterialFinder(TEXT("Material'/SimpleNav3D/M_Nav.M_Nav'"));
	checkf(MaterialFinder.Succeeded(), TEXT("Could not find grid material for SimpleNav3D plugin. Make sure the SimpleNav3D plugin is correctly installed."));

	GridMaterial = MaterialFinder.Object;
}

void AOctNavVolume3D::BeginPlay()
{
	Super::BeginPlay();

	// -------------------------------------------------------
	// Allocate NavNode array for the entire grid
	// -------------------------------------------------------
	const int32 TotalNodes = GetTotalDivisions();
	NavNodes = new NavNode[TotalNodes];

	// Precomputed neighbor offset list for 3D grid adjacency:
	//  - Above, middle, below layers.
	//  - Used to build neighbor links between NavNodes.
	static const TArray<FIntVector> NeighbourOffsets = {
		// Above (z + 1)
	   { 1, -1,  1}, { 1,  0,  1}, { 1,  1,  1},
	   { 0, -1,  1}, { 0,  0,  1}, { 0,  1,  1},
	   {-1, -1,  1}, {-1,  0,  1}, {-1,  1,  1},

	   // Middle (z)
	   { 1, -1,  0}, { 1,  0,  0}, { 1,  1,  0},
	   { 0, -1,  0},               { 0,  1,  0},
	   {-1, -1,  0}, {-1,  0,  0}, {-1,  1,  0},

	   // Below (z - 1)
	   { 1, -1, -1}, { 1,  0, -1}, { 1,  1, -1},
	   { 0, -1, -1}, { 0,  0, -1}, { 0,  1, -1},
	   {-1, -1, -1}, {-1,  0, -1}, {-1,  1, -1}
	};

	// Helper lambda: attempts to add a valid neighbour based on
	// grid bounds + MinSharedNeighborAxes constraint.
	auto AddNeighbour = [this](NavNode* Node, const FIntVector Offset)
		{
			const FIntVector NCoord = Node->Coordinates + Offset;

			// Ignore neighbours outside of the grid
			if (!this->AreCoordinatesValid(NCoord))
			{
				return;
			}

			// Count how many axes are shared with the candidate node
			int8 SharedAxes = 0;
			if (Node->Coordinates.X == NCoord.X) SharedAxes++;
			if (Node->Coordinates.Y == NCoord.Y) SharedAxes++;
			if (Node->Coordinates.Z == NCoord.Z) SharedAxes++;

			// Only connect neighbours that share enough axes (e.g., 6- or 18-connected)
			if (SharedAxes >= this->MinSharedNeighborAxes && SharedAxes < 3)
			{
				Node->Neighbours.push_back(GetNode(NCoord));
			}
		};

	// -------------------------------------------------------
	// Populate all NavNodes and build neighbour adjacency
	// -------------------------------------------------------
	for (int32 Z = 0; Z < DivisionsZ; ++Z)
		for (int32 Y = 0; Y < DivisionsY; ++Y)
			for (int32 X = 0; X < DivisionsX; ++X)
			{
				NavNode* Node = GetNode({ X, Y, Z });
				Node->Coordinates = { X, Y, Z };

				for (const FIntVector& Offset : NeighbourOffsets)
				{
					AddNeighbour(Node, Offset);
				}
			}

	// -------------------------------------------------------
	// Build octree for coarse occupancy / blockage queries
	// -------------------------------------------------------
	DestroyOctree(); // Destroy any previous tree (just in case)
	OctreeMinCellSize = FMath::Max(OctreeMinCellSize, DivisionSize);
	const FBox EntireGridBoxInWorld = GetWorldAlignedVolumeBox();
	OctreeRoot = BuildOctree(EntireGridBoxInWorld, 0, TArray<TEnumAsByte<EObjectTypeQuery>>(), nullptr);
}

void AOctNavVolume3D::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Cleanup octree and NavNode array
	DestroyOctree();

	delete[] NavNodes;
	NavNodes = nullptr;

	Super::EndPlay(EndPlayReason);
}

//
// ============================================================================
// Debug Grid Mesh Generation
// ============================================================================
//

void AOctNavVolume3D::CreateLine(const FVector& InStart, const FVector& InEnd, const FVector& InUpNormal, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles)
{
	const float HalfThickness = LineThickness * 0.5f;

	FVector LineDir = InEnd - InStart;
	if (!LineDir.Normalize())
	{
		// Degenerate line: start almost equals end; skip drawing
		return;
	}

	// Compute basis directions used to extrude a quad around the line
	FVector SideDir1 = FVector::CrossProduct(LineDir, InUpNormal);
	if (!SideDir1.Normalize())
	{
		SideDir1 = FVector::CrossProduct(LineDir, FVector::UpVector).GetSafeNormal();
	}

	FVector SideDir2 = FVector::CrossProduct(LineDir, SideDir1).GetSafeNormal();

	// Helper lambda: build a quad along ThicknessDirection
	auto AddQuad = [&OutVertices, &OutTriangles, &InStart, &InEnd, HalfThickness](const FVector& ThicknessDirection)
		{
			const int32 BaseIndex = OutVertices.Num();

			OutVertices.Add(InStart + ThicknessDirection * HalfThickness);
			OutVertices.Add(InEnd + ThicknessDirection * HalfThickness);
			OutVertices.Add(InStart - ThicknessDirection * HalfThickness);
			OutVertices.Add(InEnd - ThicknessDirection * HalfThickness);

			OutTriangles.Add(BaseIndex + 2);
			OutTriangles.Add(BaseIndex + 1);
			OutTriangles.Add(BaseIndex + 0);

			OutTriangles.Add(BaseIndex + 2);
			OutTriangles.Add(BaseIndex + 3);
			OutTriangles.Add(BaseIndex + 1);

			// Add reverse winding so the debug grid remains visible from both sides.
			OutTriangles.Add(BaseIndex + 0);
			OutTriangles.Add(BaseIndex + 1);
			OutTriangles.Add(BaseIndex + 2);

			OutTriangles.Add(BaseIndex + 1);
			OutTriangles.Add(BaseIndex + 3);
			OutTriangles.Add(BaseIndex + 2);
		};

	AddQuad(SideDir1);
	AddQuad(SideDir2);
}

//
// ============================================================================
// Grid Coordinate Helpers
// ============================================================================
//

bool AOctNavVolume3D::AreCoordinatesValid(const FIntVector& Coordinates) const
{
	return Coordinates.X >= 0 && Coordinates.X < DivisionsX
		&& Coordinates.Y >= 0 && Coordinates.Y < DivisionsY
		&& Coordinates.Z >= 0 && Coordinates.Z < DivisionsZ;
}

void AOctNavVolume3D::ClampCoordinatesToGrid(FIntVector& Coordinates)
{
	Coordinates.X = FMath::Clamp(Coordinates.X, 0, DivisionsX - 1);
	Coordinates.Y = FMath::Clamp(Coordinates.Y, 0, DivisionsY - 1);
	Coordinates.Z = FMath::Clamp(Coordinates.Z, 0, DivisionsZ - 1);
}

//
// ============================================================================
// Octree Lifetime Management
// ============================================================================
//

void AOctNavVolume3D::DestroyOctree()
{
	if (OctreeRoot)
	{
		delete OctreeRoot;
		OctreeRoot = nullptr;
	}
}

//
// ============================================================================
// Bounds and Transform Helpers
// ============================================================================
//

FBox AOctNavVolume3D::GetWorldAlignedVolumeBox()
{
	// Local grid bounds in actor space (no rotation)
	FBox LocalBox(
		FVector::ZeroVector,
		FVector(GetGridXBound(), GetGridYBound(), GetGridZBound())
	);

	// Only apply translation – ignore rotation & scale to keep grid axis-aligned to world
	const FVector WorldOffset = GetActorLocation();

	return LocalBox.ShiftBy(WorldOffset);
}

FIntVector AOctNavVolume3D::ConvertWorldLocationToGridCoordinates(const FVector& WorldCoordinate)
{
	// Transform world-space location into local grid space
	FTransform GridTransform = GetActorTransform();
	const FVector GridSpacePos = UKismetMathLibrary::InverseTransformLocation(GridTransform, WorldCoordinate);

	FIntVector GridSpaceCoords;

	GridSpaceCoords.X = FMath::Clamp(FMath::FloorToInt(GridSpacePos.X / DivisionSize), 0, DivisionsX - 1);
	GridSpaceCoords.Y = FMath::Clamp(FMath::FloorToInt(GridSpacePos.Y / DivisionSize), 0, DivisionsY - 1);
	GridSpaceCoords.Z = FMath::Clamp(FMath::FloorToInt(GridSpacePos.Z / DivisionSize), 0, DivisionsZ - 1);

	return GridSpaceCoords;
}

FVector AOctNavVolume3D::ConvertGridCoordinatesToWorldLocation(const FIntVector& GridCoordinates)
{
	FIntVector GridCoordsCopy = GridCoordinates;
	ClampCoordinatesToGrid(GridCoordsCopy);

	// Convert grid indices to local position at cell center
	FVector GridSpacePos(0.0f, 0.0f, 0.0f);
	const float EdgeToCenterOffset = DivisionSize * 0.5f;
	GridSpacePos.X = (GridCoordsCopy.X * DivisionSize) + EdgeToCenterOffset;
	GridSpacePos.Y = (GridCoordsCopy.Y * DivisionSize) + EdgeToCenterOffset;
	GridSpacePos.Z = (GridCoordsCopy.Z * DivisionSize) + EdgeToCenterOffset;

	// Transform back to world space
	return UKismetMathLibrary::TransformLocation(GetActorTransform(), GridSpacePos);
}

//
// ============================================================================
// Collision / Overlap Helpers
// ============================================================================
//

bool AOctNavVolume3D::IsActorOverlapping(
	float InAgentRadius,
	float InAgentHalfHeight,
	AActor* IgnoreActor,
	const FVector& InWorldLocation,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter) const
{
	// Build object type query params based on provided object types
	FCollisionObjectQueryParams ObjectParams;
	for (TEnumAsByte<EObjectTypeQuery> ObjType : InObjectTypes)
	{
		ObjectParams.AddObjectTypesToQuery(UEngineTypes::ConvertToCollisionChannel(ObjType));
	}

	// Configure collision query (optional ignored actor)
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(IsActorOverlappingTest), false);
	QueryParams.bFindInitialOverlaps = true;
	if (IgnoreActor)
	{
		QueryParams.AddIgnoredActor(IgnoreActor);
	}

	// Capsule representing agent
	const FCollisionShape CapsuleShape =
		FCollisionShape::MakeCapsule(InAgentRadius, InAgentHalfHeight);

	TArray<FOverlapResult> OverlapResults;
	const bool bHit = GetWorld()->OverlapMultiByObjectType(
		OverlapResults,
		InWorldLocation,
		FQuat::Identity,
		ObjectParams,
		CapsuleShape,
		QueryParams
	);

	return bHit;
}

//
// ============================================================================
// Nearest Free Node Search (BFS over grid)
// ============================================================================
//

NavNode* AOctNavVolume3D::FindNearestFreeNode(
	NavNode* InFromNode,
	AActor* IgnoredActor,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter,
	float InDetectionRadius,
	float InDetectionHalfHeight)
{
	if (!InFromNode || AreCoordinatesValid(InFromNode->Coordinates) == false)
	{
		return nullptr;
	}

	// BFS over grid nodes starting from InFromNode
	TQueue<NavNode*> Queue;
	TSet<NavNode*> Visited;

	Queue.Enqueue(InFromNode);
	Visited.Add(InFromNode);

	while (!Queue.IsEmpty())
	{
		NavNode* CurNode;
		Queue.Dequeue(CurNode);

		FVector NodeWorldLocation = ConvertGridCoordinatesToWorldLocation(CurNode->Coordinates);

		// Skip nodes marked as blocked in the octree
		if (OctreeRoot && QueryPointBlocked(NodeWorldLocation))
		{
			// Node is blocked at octree level, continue search
		}
		else
		{
			// Additional check: ensure no dynamic actor overlap for this agent/capsule
			if (!IsActorOverlapping(
				InDetectionRadius,
				InDetectionHalfHeight,
				IgnoredActor,
				NodeWorldLocation,
				InObjectTypes,
				InActorClassFilter))
			{
				return CurNode;
			}
		}

		// Enqueue neighbours that were not visited yet
		for (NavNode* Neighbour : CurNode->Neighbours)
		{
			if (!Visited.Contains(Neighbour))
			{
				Queue.Enqueue(Neighbour);
				Visited.Add(Neighbour);
			}
		}
	}

	// No free node found reachable from the starting node
	return nullptr;
}

//
// ============================================================================
// Octree Construction / Query
// ============================================================================
//

FOctreeNode* AOctNavVolume3D::BuildOctree(
	const FBox& InBox,
	int32 InDepth,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter)
{
	// Create node representing the current bounding box
	FOctreeNode* TreeNode = new FOctreeNode(InBox);

	// Leaf condition based on max side length and max depth
	const FVector BoxSize = InBox.GetSize();
	const float MaxSideLength = FMath::Max3(BoxSize.X, BoxSize.Y, BoxSize.Z);

	const bool bSmallEnough = (MaxSideLength <= OctreeMinCellSize + KINDA_SMALL_NUMBER);
	const bool bMaxDepthReached = (InDepth >= OctreeMaxDepth);

	// Leaf node: determine if this box is blocked
	if (bSmallEnough || bMaxDepthReached)
	{
		TreeNode->bIsLeaf = true;
		TreeNode->bBlocked = IsBoxBlocked(InBox, InObjectTypes, InActorClassFilter);
		return TreeNode;
	}

	// Internal node: split into 8 child boxes
	TreeNode->bIsLeaf = false;
	const FVector C = InBox.GetCenter();
	const FVector Min = InBox.Min;
	const FVector Max = InBox.Max;

	FBox ChildBoxes[8] = {
		FBox(FVector(Min.X, Min.Y, Min.Z), FVector(C.X,   C.Y,   C.Z)),   // 0
		FBox(FVector(C.X,  Min.Y, Min.Z), FVector(Max.X,  C.Y,   C.Z)),   // 1
		FBox(FVector(Min.X, C.Y,  Min.Z), FVector(C.X,    Max.Y,  C.Z)),  // 2
		FBox(FVector(C.X,  C.Y,  Min.Z), FVector(Max.X,   Max.Y,  C.Z)),  // 3
		FBox(FVector(Min.X, Min.Y, C.Z), FVector(C.X,     C.Y,    Max.Z)),// 4
		FBox(FVector(C.X,  Min.Y, C.Z), FVector(Max.X,    C.Y,    Max.Z)),// 5
		FBox(FVector(Min.X, C.Y,  C.Z), FVector(C.X,      Max.Y,  Max.Z)),// 6
		FBox(FVector(C.X,  C.Y,  C.Z), FVector(Max.X,     Max.Y,  Max.Z)) // 7
	};

	bool bAllBlocked = true;
	for (int i = 0; i < 8; ++i)
	{
		TreeNode->Children[i] = BuildOctree(ChildBoxes[i], InDepth + 1, InObjectTypes, InActorClassFilter);
		bAllBlocked &= (TreeNode->Children[i]->bIsLeaf && TreeNode->Children[i]->bBlocked);
	}

	// Static-region compression: if every child is already a blocked leaf,
	// collapse this branch into one blocked leaf to reduce memory and query depth.
	if (bAllBlocked)
	{
		TreeNode->bIsLeaf = true;
		TreeNode->bBlocked = true;

		for (FOctreeNode*& Child : TreeNode->Children)
		{
			delete Child;
			Child = nullptr;
		}
	}

	return TreeNode;
}

bool AOctNavVolume3D::IsBoxBlocked(
	const FBox& InBox,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter)
{
	// Build list of collision channels to query against
	FCollisionObjectQueryParams ObjectCollisionParams;
	for (auto ObjectType : InObjectTypes)
	{
		ObjectCollisionParams.AddObjectTypesToQuery(UEngineTypes::ConvertToCollisionChannel(ObjectType));
	}

	// Compute center and extent for box collision test
	const FVector Center = InBox.GetCenter();
	const FVector Extent = InBox.GetExtent();

	// Check if the box overlaps with any relevant objects
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(OctreeBoxTest), false);
	bool bHit = GetWorld()->OverlapAnyTestByObjectType(
		Center,
		FQuat::Identity,
		ObjectCollisionParams,
		FCollisionShape::MakeBox(Extent),
		QueryParams);

	return bHit;

	// TODO: Optional refinement: trace for "blocked" only, instead of any overlap.
}

bool AOctNavVolume3D::QueryPointBlocked(const FVector& WorldPoint) const
{
	if (!OctreeRoot)
	{
		return false;
	}

	// Traverse octree down to the leaf that contains WorldPoint
	const FOctreeNode* CurrentNode = OctreeRoot;
	while (CurrentNode && !CurrentNode->bIsLeaf)
	{
		const FVector Center = CurrentNode->Bounds.GetCenter();
		const bool bIsHighX = WorldPoint.X >= Center.X;
		const bool bIsHighY = WorldPoint.Y >= Center.Y;
		const bool bIsHighZ = WorldPoint.Z >= Center.Z;

		const int CorrectChildIndex =
			(bIsHighX ? 1 : 0)
			| ((bIsHighY ? 1 : 0) << 1)
			| ((bIsHighZ ? 1 : 0) << 2);

		CurrentNode = CurrentNode->Children[CorrectChildIndex];
	}

	return (CurrentNode) ? CurrentNode->bBlocked : false;
}

//
// ============================================================================
// Editor Construction – Build Debug Grid Mesh
// ============================================================================
//

void AOctNavVolume3D::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

#if WITH_EDITOR
	// Enforce axis-aligned, unit-scale volume for simplicity
	if (!Transform.GetRotation().IsIdentity())
	{
		UE_LOG(LogTemp, Warning, TEXT("OctNavVolume3D: Rotation is ignored. Please keep this actor unrotated."));
	}

	if (!Transform.GetScale3D().Equals(FVector(1.f, 1.f, 1.f), KINDA_SMALL_NUMBER))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("OctNavVolume3D: Scale is ignored. Please keep scale = (1,1,1)."));
	}
#endif

	TArray<FVector> Vertices;
	TArray<int32> TrianglesIndexArray;

	// Estimated number of grid lines for all three planes (XY, YZ, XZ)
	const uint32 EstimateLineCount =
		(DivisionsX + 1) * (DivisionsY + 1) * 2   // XY planes
		+ (DivisionsY + 1) * (DivisionsZ + 1) * 2 // YZ planes
		+ (DivisionsX + 1) * (DivisionsZ + 1) * 2; // XZ planes

	const uint32 EstimateVertexCount = EstimateLineCount * 4;
	const uint32 EstimateTriangleIndexCount = EstimateLineCount * 6;

	Vertices.Reserve(EstimateVertexCount);
	TrianglesIndexArray.Reserve(EstimateTriangleIndexCount);

	const float GridXBound = GetGridXBound();
	const float GridYBound = GetGridYBound();
	const float GridZBound = GetGridZBound();

	// Helper to add one thick line segment to the procedural mesh
	auto AddLine = [this, &Vertices, &TrianglesIndexArray](const FVector& InStart, const FVector& InEnd, const FVector& InUpNormal)
		{
			this->CreateLine(InStart, InEnd, InUpNormal, Vertices, TrianglesIndexArray);
		};

	FVector Start = FVector::ZeroVector;
	FVector End = FVector::ZeroVector;

	// -------------------------------------------------------
	// Lines parallel to Y axis (varying X, Z fixed)
	// -------------------------------------------------------
	for (int32 Z = 0; Z <= DivisionsZ; ++Z)
	{
		Start.Z = DivisionSize * Z;
		End.Z = DivisionSize * Z;

		for (int32 X = 0; X <= DivisionsX; ++X)
		{
			Start.X = X * DivisionSize;
			End.X = X * DivisionSize;

			Start.Y = 0.0f;
			End.Y = GridYBound;

			AddLine(Start, End, FVector::UpVector);
		}
	}

	// -------------------------------------------------------
	// Lines parallel to X axis (varying Y, Z fixed)
	// -------------------------------------------------------
	for (int32 Z = 0; Z <= DivisionsZ; ++Z)
	{
		Start.Z = DivisionSize * Z;
		End.Z = DivisionSize * Z;

		for (int32 Y = 0; Y <= DivisionsY; ++Y)
		{
			Start.Y = Y * DivisionSize;
			End.Y = Y * DivisionSize;

			Start.X = 0.0f;
			End.X = GridXBound;

			AddLine(Start, End, FVector::UpVector);
		}
	}

	// -------------------------------------------------------
	// Lines parallel to Z axis (varying Z, X/Y fixed)
	// -------------------------------------------------------
	for (int32 X = 0; X <= DivisionsX; ++X)
	{
		Start.X = End.X = X * DivisionSize;

		for (int32 Y = 0; Y <= DivisionsY; ++Y)
		{
			Start.Y = End.Y = Y * DivisionSize;

			Start.Z = 0.f;
			End.Z = GridZBound;

			AddLine(Start, End, FVector::ForwardVector);
		}
	}

	// Build the procedural mesh section for the debug grid
	ProceduralMeshComponent->CreateMeshSection(
		0,
		Vertices,                     // Vertices
		TrianglesIndexArray,          // Indices
		TArray<FVector>(),            // Normals
		TArray<FVector2D>(),          // UVs
		TArray<FColor>(),             // Vertex Colors
		TArray<FProcMeshTangent>(),   // Tangents
		false                         // bCreateCollision
	);

	// Apply dynamic material instance to control color/opacity at runtime
	if (GridMaterial)
	{
		UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(GridMaterial, this);

		if (MID)
		{
			MID->SetVectorParameterValue(TEXT("Color"), Color);
			MID->SetScalarParameterValue(TEXT("Opacity"), Color.A);
			ProceduralMeshComponent->SetMaterial(0, MID);
		}
	}
}

void AOctNavVolume3D::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Currently no per-frame behavior needed for the volume.
	// Hook for future dynamic visualization or debug updates.
}

//
// ============================================================================
// Node Access
// ============================================================================
//

NavNode* AOctNavVolume3D::GetNode(FIntVector Coordinates)
{
	// Clamp coordinates into valid grid range
	ClampCoordinatesToGrid(Coordinates);

	const int32 DivisionPerLevel = DivisionsX * DivisionsY;
	const int32 Index =
		(Coordinates.Z * DivisionPerLevel) +
		(Coordinates.Y * DivisionsX) +
		Coordinates.X;

	if (Index < 0 || Index >= GetTotalDivisions())
	{
		return nullptr;
	}
	return &NavNodes[Index];
}

//
// ============================================================================
// A* Pathfinding on 3D Grid
// ============================================================================
//

bool AOctNavVolume3D::FindPath(
	const FVector& InStart,
	const FVector& InDestination,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter,
	TArray<FVector>& OutPath,
	AActor* InActor /*= nullptr */,
	float InDetectionRadius /*= 34.f*/,
	float InDetectionHalfHeight /*= 44.f */)
{
	OutPath.Reset();

	// Convert world-space start/destination to grid nodes
	NavNode* StartNode = GetNode(ConvertWorldLocationToGridCoordinates(InStart));
	NavNode* GoalNode = GetNode(ConvertWorldLocationToGridCoordinates(InDestination));

	if (!StartNode || !GoalNode)
	{
#if WITH_EDITOR
		UE_LOG(LogTemp, Warning, TEXT("Start or End node not found"));
#endif
		return false;
	}

	bool HasGoalFinalized = false;

	// -------------------------------------------------------
	// Snap goal to nearest free node if original goal is blocked
	// (by static geometry via octree and/or dynamic overlap)
	// -------------------------------------------------------
	if (OctreeRoot && QueryPointBlocked(ConvertGridCoordinatesToWorldLocation(GoalNode->Coordinates)))
	{
		if (NavNode* NewGoal = FindNearestFreeNode(
			GoalNode,
			InActor,
			InObjectTypes,
			InActorClassFilter,
			InDetectionRadius,
			InDetectionHalfHeight))
		{
			GoalNode = NewGoal;
			HasGoalFinalized = true;
		}
		else
		{
			if (GEngine)
			{
				GEngine->AddOnScreenDebugMessage(
					-1, 5.f, FColor::Red,
					TEXT("No free goal node found near destination"));
			}
			return false;
		}
	}

	// If goal node is occupied by dynamic actors, also try to relocate it
	if (!HasGoalFinalized && IsActorOverlapping(
		InDetectionRadius,
		InDetectionHalfHeight,
		InActor,
		ConvertGridCoordinatesToWorldLocation(GoalNode->Coordinates),
		InObjectTypes,
		InActorClassFilter))
	{
		if (NavNode* NewGoal = FindNearestFreeNode(
			GoalNode,
			InActor,
			InObjectTypes,
			InActorClassFilter,
			InDetectionRadius,
			InDetectionHalfHeight))
		{
			GoalNode = NewGoal;
		}
		else
		{
			if (GEngine)
			{
				GEngine->AddOnScreenDebugMessage(
					-1, 5.f, FColor::Red,
					TEXT("No free goal node found near destination"));
			}
			return false;
		}
	}

	// -------------------------------------------------------
	// A* Setup
	// -------------------------------------------------------
	std::priority_queue<NavNode*, std::vector<NavNode*>, NavNodeCompare> OpenSet;
	std::unordered_map<NavNode*, NavNode*> CameFrom;
	std::unordered_map<NavNode*, float> GScores;
	std::unordered_set<NavNode*> Visited;

	// Heuristic: Euclidean distance in grid space
	auto GetHeuristic = [&GoalNode](NavNode* InNode)
		{
			return FVector::Distance(FVector(InNode->Coordinates), FVector(GoalNode->Coordinates));
		};

	// Actual edge cost: Euclidean distance between neighbour cells
	auto GetDistance = [](NavNode* FromNode, NavNode* ToNode)
		{
			return FVector::Distance(FVector(FromNode->Coordinates), FVector(ToNode->Coordinates));
		};

	// Safe accessor for g-scores (defaults to "infinite" if not set)
	auto GetGScore = [&GScores](NavNode* InNode)
		{
			auto It = GScores.find(InNode);
			if (It != GScores.end())
			{
				return It->second;
			}
			return FLT_MAX;
		};

	// Initialize start node
	StartNode->FScore = GetHeuristic(StartNode);
	OpenSet.push(StartNode);
	GScores[StartNode] = 0.0f;

	// -------------------------------------------------------
	// A* Main Loop
	// -------------------------------------------------------
	while (!OpenSet.empty())
	{
		NavNode* CurrentNavNode = OpenSet.top();
		OpenSet.pop();
		Visited.insert(CurrentNavNode);

		// Goal reached: reconstruct path
		if (CurrentNavNode == GoalNode)
		{
			OutPath.Add(ConvertGridCoordinatesToWorldLocation(CurrentNavNode->Coordinates));

			while (CameFrom.contains(CurrentNavNode))
			{
				OutPath.Insert(
					ConvertGridCoordinatesToWorldLocation(CurrentNavNode->Coordinates),
					0);
				CurrentNavNode = CameFrom[CurrentNavNode];
			}

			OutPath.Insert(ConvertGridCoordinatesToWorldLocation(StartNode->Coordinates), 0);
			return true;
		}

		const float CurrentGScore = GetGScore(CurrentNavNode);

		// Evaluate neighbours
		for (NavNode* Neighbour : CurrentNavNode->Neighbours)
		{
			const FVector NeighbourWorldPos = ConvertGridCoordinatesToWorldLocation(Neighbour->Coordinates);

			// Skip neighbours blocked by octree
			if (OctreeRoot && QueryPointBlocked(NeighbourWorldPos))
			{
				continue;
			}

			const float TentativeG = CurrentGScore + GetDistance(CurrentNavNode, Neighbour);
			const float ExistingScore = GetGScore(Neighbour);

			// Found a cheaper path to this neighbour
			if (TentativeG < ExistingScore)
			{
				// Check dynamic overlaps (e.g., other actors or obstacles)
				if (IsActorOverlapping(
					InDetectionRadius,
					InDetectionHalfHeight,
					InActor,
					NeighbourWorldPos,
					InObjectTypes,
					InActorClassFilter))
				{
					continue;
				}

				CameFrom[Neighbour] = CurrentNavNode;
				GScores[Neighbour] = TentativeG;

				Neighbour->FScore = TentativeG + GetHeuristic(Neighbour);

				if (!Visited.contains(Neighbour))
				{
					OpenSet.push(Neighbour);
				}
			}
		}
	}

	// No path found
	return false;
}
