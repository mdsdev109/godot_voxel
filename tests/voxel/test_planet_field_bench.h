#ifndef VOXEL_TEST_PLANET_FIELD_BENCH_H
#define VOXEL_TEST_PLANET_FIELD_BENCH_H

namespace zylann::voxel::tests {

// Measures `VoxelGeneratorGraph::generate_series` throughput at the batch sizes a physics narrow phase
// would use (tens of samples), against the batch sizes block generation uses (thousands).
//
// The question this answers: is the per-call setup cost of the graph runtime small enough that an
// analytic planet collision shape can call it directly from the narrow phase, or must the hot path be
// hand-written C++ with the graph kept only for range analysis and shader generation?
//
// Run with: <godot binary> --headless --quit --run_planet_field_bench
//
// The bench runs during module initialization, so without --quit the engine goes on to boot a
// headless project manager and never exits.
void run_planet_field_bench();

} // namespace zylann::voxel::tests

#endif // VOXEL_TEST_PLANET_FIELD_BENCH_H
