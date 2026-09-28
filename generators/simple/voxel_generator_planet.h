#ifndef VOXEL_GENERATOR_PLANET_H
#define VOXEL_GENERATOR_PLANET_H

#include "../../storage/voxel_buffer.h"
#include "../../util/thread/rw_lock.h"
#include "../voxel_generator.h"

#include "core/math/planet_field.h"
#include "core/math/transform_3d.h"
#include "scene/resources/3d/planet_field_3d.h"

namespace zylann::voxel {

// Fills the SDF channel from the planet field, for a voxel volume placed somewhere on the planet: a dig
// site (planet-demo docs/DIGGING.md). `voxel_to_planet` maps voxel coordinates -- the terrain node's
// local space, one unit per voxel -- to planet coordinates, including the voxel size in its basis. The
// value stored is |p| - f(p / |p|) in voxels: negative below the ground, positive above, and zero
// exactly on the analytic surface, so an undug site is the planet, sampled.
//
// The field evaluates identically on every machine, and so does this, so voxels generated here -- and
// any edits replayed on top -- are the same everywhere; only the edits need to be sent or saved.
class VoxelGeneratorPlanet : public VoxelGenerator {
	GDCLASS(VoxelGeneratorPlanet, VoxelGenerator)

public:
	VoxelGeneratorPlanet();
	~VoxelGeneratorPlanet();

	int get_used_channels_mask() const override;
	Result generate_block(VoxelGenerator::VoxelQueryData input) override;

	void set_field(const Ref<PlanetField3D> &p_field);
	Ref<PlanetField3D> get_field() const;

	void set_voxel_to_planet(const Transform3D &p_transform);
	Transform3D get_voxel_to_planet() const;

	// The SDF of one point in voxel space, as generate_block computes it. For tests and for tools that
	// need the undug surface without a terrain.
	float get_sdf_at(const Vector3 &p_voxel_position) const;

	// Dig sites sit on a grid of cube-sphere lattice cells, `p_resolution` per face edge. These place
	// them, with PlanetLattice's deterministic mapping, so every machine puts a site in exactly the same
	// place -- which the voxels replayed on top of it depend on.
	//
	// The cell (face, i, j) containing a planet-space point.
	static Vector3i site_key_of(const Vector3 &p_point, int p_resolution);
	// Voxel space to planet space for the site in cell (face, i, j): x along the cell's i direction,
	// y up, z = x cross y; one unit per voxel, the voxel edge chosen so `p_voxels_across` voxels span
	// the cell; voxel (0, 0, 0) at a corner of the footprint, `p_voxels_below` voxels under the surface
	// at the cell's centre.
	static Transform3D make_site_transform(const Ref<PlanetField3D> &p_field, const Vector3i &p_key, int p_resolution,
			int p_voxels_across, int p_voxels_below);

	// Carves: spheres removed from the ground, in voxel coordinates. A voxel's value is
	// max(field, radius - distance) over every carve whose influence box -- the sphere's bounding cube
	// grown by CARVE_MARGIN voxels -- contains it. Plain IEEE arithmetic and an exactly commutative max,
	// so a site's voxels are a pure function of the field and its set of carves: independent of the
	// order carves arrive in, of how often one is applied, and of the machine.
	//
	// (godot_voxel's own do_sphere is none of those things: it computes lerp(a, max(a, -b), strength),
	// and a + (x - a) is not x in floating point, so the result depends on the value it started from.)
	static constexpr int CARVE_MARGIN = 2;
	void add_carve(const Vector3 &p_centre, double p_radius);
	void clear_carves();
	int get_carve_count() const;
	// The voxel box a carve can change, for regenerating it: [position, size].
	static AABB get_carve_box(const Vector3 &p_centre, double p_radius);

protected:
	static void _bind_methods();

private:
	struct Parameters {
		PlanetFieldData field;
		bool has_field = false;
		Transform3D voxel_to_planet;
		double inv_voxel_size = 1.0;
		LocalVector<Vector3> carve_centres;
		LocalVector<double> carve_radii;
	};

	Ref<PlanetField3D> _field;
	Parameters _parameters;
	RWLock _parameters_lock;
};

} // namespace zylann::voxel

#endif // VOXEL_GENERATOR_PLANET_H
