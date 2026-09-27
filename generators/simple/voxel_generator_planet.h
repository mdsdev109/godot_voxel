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

protected:
	static void _bind_methods();

private:
	struct Parameters {
		PlanetFieldData field;
		bool has_field = false;
		Transform3D voxel_to_planet;
		double inv_voxel_size = 1.0;
	};

	Ref<PlanetField3D> _field;
	Parameters _parameters;
	RWLock _parameters_lock;
};

} // namespace zylann::voxel

#endif // VOXEL_GENERATOR_PLANET_H
