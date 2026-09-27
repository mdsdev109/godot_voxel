#ifndef VOXEL_TEST_PLANET_FIELD_CHECK_H
#define VOXEL_TEST_PLANET_FIELD_CHECK_H

namespace zylann::voxel::tests {

// Phase 2 acceptance checks for the analytic planet field.
//
// The design's load-bearing claim is that the collider, the voxel generator and the renderer all
// evaluate one `f`. Two of those three live on the CPU and can be compared exactly here; the third is
// the GLSL twin, which this cannot reach and which stays a review obligation.
//
// Checks:
//   1. Scalar fbm_3d and batched fbm_3d_series agree bit for bit (catches the vectorized path
//      reassociating and drifting from the reference).
//   2. PlanetField3D::eval_radius and eval_radius_batch agree bit for bit.
//   3. The voxel graph's PlanetNoise3D node agrees with the same noise called directly, so the
//      generator and the collider cannot diverge.
//   4. Global radius bounds contain f. These are exact by construction, so any failure is a real bug.
//   5. Cone bounds contain f over the cone. Unlike (4) these rest on a measured Lipschitz constant.
//   6. The Lipschitz constant is re-measured and checked against the value the bounds assume.
//
// Run with: <godot binary> --headless --quit --run_planet_field_check
void run_planet_field_check();

// Prints a digest of f over a fixed set of directions, one per layer of the field, so two builds or two
// machines can be compared bit for bit. Not a check: it has nothing to compare against on its own.
// Set PLANET_FIELD_DIGEST_OUT to also write the final layer's radii as raw doubles.
//
// Run with: <godot binary> --headless --quit --run_planet_field_digest
void run_planet_field_digest();

} // namespace zylann::voxel::tests

#endif // VOXEL_TEST_PLANET_FIELD_CHECK_H
