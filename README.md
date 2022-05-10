[![Actions Status](https://github.com/0xFounders/divert/workflows/Continuous%20integration/badge.svg)](https://github.com/0xFounders/divert/actions)
[![Crate](https://img.shields.io/crates/v/divert.svg)](https://crates.io/crates/divert)

# Divert
Rust bindings for [Recast Navigation](https://github.com/recastnavigation/recastnavigation).


## Purpose
Provide safe bindings to [Recast Navigation](https://github.com/recastnavigation/recastnavigation) to allow for 3d navigation in a rust application.

## Overview
### `src/extern.cpp`
C function definitions wrapping Detour C++ Functions.
### `src/binding.rs`
Rust bindings to the C functions exposed in `src/extern.cpp`.
### `src/lib.rs`
Safe Rust abstractions of Detour components e.g ensuring correct freeing of DtNavMesh and DtNavMeshQuery.

## Use Case
Refer to `examples/pathfinding.rs` for a demonstration of loading geometry generated with [Trinity Core](https://github.com/TrinityCore/TrinityCore). In the below, Proof of Concept, section the paths generated are projected to in-game space. In this repository the resources for generating paths is provided, but drawing/projecting points in the game is not in scope of this project. No questions or issues should be opened requesting help or information about video game specific applications.


```
cargo run --example pathfinding
   Compiling divert v0.1.0 (...\divert)
    Finished dev [unoptimized + debuginfo] target(s) in 0.99s
     Running `target\debug\examples\pathfinding.exe`
 INFO  pathfinding > [FindPath] Loading Tile (35, 22)
 WARN  pathfinding > [FindPath] Skipping Loading of End Tile (35, 22)
 INFO  pathfinding > Smooth Path Len: 60
 INFO  pathfinding > DtVector { y: 5289.2, z: 1.6113697, x: -1910.12 }
 INFO  pathfinding > DtVector { y: 5285.3105, z: 2.4410567, x: -1911.0527 }
 INFO  pathfinding > DtVector { y: 5281.421, z: 2.7707438, x: -1911.9855 }
 ...
 INFO  pathfinding > DtVector { y: 5097.9536, z: 5.4801526, x: -1934.1309 }
 INFO  pathfinding > DtVector { y: 5099.05, z: 6.0619655, x: -1931.9 }
 WARN  pathfinding > [FindPath] Skipping Loading of Start Tile (35, 22)
 INFO  pathfinding > [FindPath] Loading Tile (35, 23)
 INFO  pathfinding > Smooth Path Len: 54
 INFO  pathfinding > DtVector { y: 4893.65, z: 2.3657227, x: -1916.64 }
 ...
 INFO  pathfinding > DtVector { y: 4691.799, z: -1.4676135, x: -1947.7577 }
 INFO  pathfinding > DtVector { y: 4687.8105, z: -1.4020469, x: -1947.4502 }
```

## Proof Of Concept
Demonstration of my independent work using this library to generate navigation paths in a third party video game.

![Navigation Demo 1](resources/docs/demo_nav.PNG)

![Navigation Demo 2](resources/docs/demo_nav_2.PNG)