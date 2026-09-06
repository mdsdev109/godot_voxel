# godot_voxel fork — agent entry point

A fork of `Zylann/godot_voxel`, vendored inside the Godot fork at `modules/voxel` but with its
**own git history**. The containing engine repository gitignores this directory: commit voxel
changes here, on branch `planet`. Do not push.

Upstream's own documentation is in `README.md` and `doc/`; this file only covers the fork.

## Why this fork exists

The game it serves is at `C:\Users\mschw\Home\Godot\Projects\planet-demo` — planets at
realistic scale (6,371,000 m), where the surface is a pure function `r = f(û)` rather than a
mesh, and the collider samples that function directly. Read that project's
`docs/ORIENTATION.md` and `docs/ARCHITECTURE.md` for the whole picture.

Voxels are meant to be the *editable* part of a world that is otherwise analytic: holes dug
into the terrain, and player-built structures and ships. **None of that is wired up yet** —
nothing in the game references a voxel class. What exists here is groundwork.

## What the fork adds

**`NODE_PLANET_NOISE_3D`** (`generators/graph/nodes/noise.h`) — a graph node that calls
`PlanetHashNoise::fbm_3d_series` from the engine's `core/math/` directly, and emits the
matching GLSL through the shared shader source. This is the point: a voxel edit and the
analytic surface around it evaluate **the same function**, so an edited region abuts the
untouched planet exactly instead of nearly.

FastNoise2 is about 4x faster and is deliberately not used — it has no `shader_gen_func` at
all, so it cannot give the collider, the generator and the renderer one `f`. This was measured.

**`VoxelLodTerrainMultiplayerSynchronizer`** (`terrain/variable_lod/`) — streams blocks to
peers by their clipbox view, with an edit journal in `VoxelData`, per-block network state on
`VoxelMeshBlock`, and a request RPC for blocks a client is missing. It reuses the set the
clipbox streaming update already computes, so a client's received blocks match the LOD it
would have loaded locally. Unused by the game so far.

**Planet field tests** (`tests/voxel/test_planet_field_{check,bench}.cpp`) — these cover
`core/math/planet_field.h`, not voxel storage. They live here because this module is what
`voxel_tests=yes` builds. Run from the engine root:

```
bin\godot.windows.editor.double.x86_64.llvm.mono.console.exe --headless --quit --run_planet_field_check
bin\godot.windows.editor.double.x86_64.llvm.mono.console.exe --headless --quit --run_planet_field_bench
```

`--quit` is required — they run at module init, and without it the engine boots a headless
project manager afterwards and never exits. Never benchmark on a `dev` build; it is `-O0`.

## Constraints

- The check compares scalar, batched and graph evaluation **bit for bit**. Approximate
  agreement is not enough, and it has already caught two defects that would have put a step at
  every chunk boundary.
- The custom noise lives in the engine's `core/math/`, not here — moved out so the Jolt shape
  does not depend on an optional module. The consequence: **godot_voxel can no longer build as
  a standalone GDExtension** without a shim. Keep that in mind before assuming an upstream
  build path still works.
- Upstream merges are expected. Keep fork changes tight and commented so a rebase is legible.
