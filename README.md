# SimpleNav3D - Unreal Engine 5 3D Pathfinding Demo

SimpleNav3D is a small Unreal Engine 5.6 C++ project focused on one gameplay-system problem: 3D pathfinding through a volumetric grid.

The repo contains a drop-in `SimpleNav3D` plugin plus a minimal Third Person template project that opens directly into a demo map. The demo map contains obstacles, a 3D navigation volume, an agent, and a destination marker. Press Play and the demo agent asks the plugin for a path, draws the path, and follows it.

## Demo

<a href="https://youtu.be/LV2WPoSnrL8">
  <img src="https://img.youtube.com/vi/LV2WPoSnrL8/maxresdefault.jpg" alt="SimpleNav3D demo video" width="720">
</a>

[Watch the demo on YouTube](https://youtu.be/LV2WPoSnrL8)


## Core Idea

`AOctNavVolume3D` builds a regular 3D grid inside an actor volume. Each cell is represented by a `NavNode`, and neighbouring cells are linked into a graph. At runtime, `FindPath` converts world positions to grid nodes, checks blocked regions, runs A*, and returns world-space waypoints.

The plugin also builds an octree over the volume for coarse blocker queries, then uses Unreal collision overlap checks to reject occupied cells for the requested agent size.

## Main Features

- 3D grid navigation volume with configurable `DivisionsX`, `DivisionsY`, `DivisionsZ`, and `DivisionSize`.
- A* pathfinding over a custom 3D node graph.
- BFS nearest-free-node search when the goal is inside the blocked space.
- Octree-backed blocked-cell queries.
- Runtime debug grid rendering with `UProceduralMeshComponent`.
- C++ and Blueprint-callable pathfinding API.

## Public API

```cpp
bool FindPath(
    const FVector& InStart,
    const FVector& InDestination,
    const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
    UClass* InActorClassFilter,
    TArray<FVector>& OutPath,
    AActor* InActor = nullptr,
    float InDetectionRadius = 34.f,
    float InDetectionHalfHeight = 44.f
);
```

Example use:

```cpp
TArray<FVector> Path;
if (NavVolume->FindPath(StartLocation, TargetLocation, ObjectTypes, nullptr, Path, Agent))
{
    // Follow the returned world-space waypoints.
}
```

## Running The Demo

1. Clone the repository.
2. Make sure Git LFS files are pulled:

   ```bash
   git lfs pull
   ```

3. Open `PluginTest.uproject` with Unreal Engine 5.6.
4. If prompted, rebuild project modules.
5. The project opens to `/Game/Nav3D_Demo`.
6. Press Play.

If the map does not open automatically, open `Content/Nav3D_Demo.umap` manually.

## Notes
- `SimpleNav3D` is a gameplay/AI programming sample, not a replacement for Unreal's built-in NavMesh.
- The plugin is useful for flying, swimming, zero-gravity, or other movement where a ground-only navmesh is not the right representation.
