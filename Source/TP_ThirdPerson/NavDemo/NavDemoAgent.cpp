#include "NavDemo/NavDemoAgent.h"

#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "OctNavVolume3D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "TimerManager.h"
#include "UObject/ConstructorHelpers.h"

ANavDemoAgent::ANavDemoAgent()
{
	PrimaryActorTick.bCanEverTick = true;

	MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	SetRootComponent(MeshComponent);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	MeshComponent->SetGenerateOverlapEvents(false);
	MeshComponent->SetRelativeScale3D(FVector(0.55f));

	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (SphereMesh.Succeeded())
	{
		MeshComponent->SetStaticMesh(SphereMesh.Object);
	}

	ObstacleObjectTypes.Add(UEngineTypes::ConvertToObjectType(ECC_WorldStatic));
}

void ANavDemoAgent::BeginPlay()
{
	Super::BeginPlay();

	if (!NavVolume)
	{
		for (TActorIterator<AOctNavVolume3D> It(GetWorld()); It; ++It)
		{
			NavVolume = *It;
			break;
		}
	}

	GetWorldTimerManager().SetTimerForNextTick(this, &ANavDemoAgent::RequestPath);
}

void ANavDemoAgent::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!CurrentPath.IsValidIndex(CurrentPathIndex))
	{
		return;
	}

	const FVector CurrentLocation = GetActorLocation();
	const FVector TargetLocation = CurrentPath[CurrentPathIndex];
	const FVector ToTarget = TargetLocation - CurrentLocation;
	const float DistanceToTarget = ToTarget.Size();

	if (DistanceToTarget <= AcceptanceRadius)
	{
		++CurrentPathIndex;
		if (!CurrentPath.IsValidIndex(CurrentPathIndex))
		{
			SetActorLocation(TargetLocation);
			return;
		}
	}

	const FVector NewLocation = FMath::VInterpConstantTo(
		CurrentLocation,
		CurrentPath[CurrentPathIndex],
		DeltaSeconds,
		MovementSpeed);

	SetActorLocation(NewLocation);
}

void ANavDemoAgent::RequestPath()
{
	CurrentPath.Reset();
	CurrentPathIndex = INDEX_NONE;

	if (!NavVolume || !DestinationActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("NavDemoAgent needs both NavVolume and DestinationActor assigned."));
		return;
	}

	const bool bFoundPath = NavVolume->FindPath(
		GetActorLocation(),
		DestinationActor->GetActorLocation(),
		ObstacleObjectTypes,
		nullptr,
		CurrentPath,
		this,
		AgentRadius,
		AgentHalfHeight);

	if (!bFoundPath || CurrentPath.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("NavDemoAgent could not find a path to the destination."));
		return;
	}

	CurrentPathIndex = 0;
	if (FVector::DistSquared(CurrentPath[0], GetActorLocation()) <= FMath::Square(AcceptanceRadius))
	{
		CurrentPathIndex = CurrentPath.Num() > 1 ? 1 : INDEX_NONE;
	}

	if (bDrawPath)
	{
		DrawCurrentPath();
	}
}

void ANavDemoAgent::DrawCurrentPath() const
{
	if (!GetWorld() || CurrentPath.Num() < 2)
	{
		return;
	}

	for (int32 Index = 0; Index < CurrentPath.Num() - 1; ++Index)
	{
		DrawDebugLine(GetWorld(), CurrentPath[Index], CurrentPath[Index + 1], FColor::Cyan, true, 30.0f, 0, 8.0f);
		DrawDebugSphere(GetWorld(), CurrentPath[Index], 18.0f, 8, FColor::Cyan, true, 30.0f);
	}

	DrawDebugSphere(GetWorld(), CurrentPath.Last(), 26.0f, 12, FColor::Green, true, 30.0f);
}
