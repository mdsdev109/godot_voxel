#include "voxel_generator_planet.h"
#include "../../util/containers/std_vector.h"

#ifdef ZN_GODOT
#include "../../util/godot/core/class_db.h"
#endif

namespace zylann::voxel {

VoxelGeneratorPlanet::VoxelGeneratorPlanet() {}

VoxelGeneratorPlanet::~VoxelGeneratorPlanet() {}

int VoxelGeneratorPlanet::get_used_channels_mask() const {
	return 1 << VoxelBuffer::CHANNEL_SDF;
}

void VoxelGeneratorPlanet::set_field(const Ref<PlanetField3D> &p_field) {
	{
		RWLockWrite wlock(_parameters_lock);
		_field = p_field;
		// A copy, so generation threads never read a field being edited on the main thread.
		_parameters.has_field = p_field.is_valid();
		if (_parameters.has_field) {
			_parameters.field = p_field->get_data();
		}
	}
	emit_changed();
}

Ref<PlanetField3D> VoxelGeneratorPlanet::get_field() const {
	RWLockRead rlock(_parameters_lock);
	return _field;
}

void VoxelGeneratorPlanet::set_voxel_to_planet(const Transform3D &p_transform) {
	{
		RWLockWrite wlock(_parameters_lock);
		_parameters.voxel_to_planet = p_transform;
		// The voxel edge in metres, from the basis. Sites are uniformly scaled.
		const double size = p_transform.basis.get_column(0).length();
		_parameters.inv_voxel_size = size > 0.0 ? 1.0 / size : 1.0;
	}
	emit_changed();
}

Transform3D VoxelGeneratorPlanet::get_voxel_to_planet() const {
	RWLockRead rlock(_parameters_lock);
	return _parameters.voxel_to_planet;
}

float VoxelGeneratorPlanet::get_sdf_at(const Vector3 &p_voxel_position) const {
	Parameters params;
	{
		RWLockRead rlock(_parameters_lock);
		params = _parameters;
	}
	if (!params.has_field) {
		return 1.f;
	}
	const Vector3 p = params.voxel_to_planet.xform(p_voxel_position);
	const double r = p.length();
	return float((r - params.field.eval_radius(p / r)) * params.inv_voxel_size);
}

VoxelGenerator::Result VoxelGeneratorPlanet::generate_block(VoxelGenerator::VoxelQueryData input) {
	Result result;
	Parameters params;
	{
		RWLockRead rlock(_parameters_lock);
		params = _parameters;
	}
	VoxelBuffer &out_buffer = input.voxel_buffer;
	if (!params.has_field) {
		out_buffer.clear_channel_f(VoxelBuffer::CHANNEL_SDF, 1.f);
		return result;
	}

	const Vector3i origin = input.origin_in_voxels;
	const Vector3i bs = out_buffer.get_size();
	const int stride = 1 << input.lod;
	const uint32_t count = uint32_t(bs.x) * uint32_t(bs.y) * uint32_t(bs.z);

	// One batched evaluation per block: the batched path is bit-identical to the scalar one (the
	// field check proves it) and several times faster.
	StdVector<Vector3> dirs(count);
	StdVector<double> radii(count);
	StdVector<double> planet_r(count);
	StdVector<double> sx(count);
	StdVector<double> sy(count);
	StdVector<double> sz(count);
	StdVector<float> noise(count);
	uint32_t i = 0;
	for (int z = 0; z < bs.z; ++z) {
		for (int x = 0; x < bs.x; ++x) {
			for (int y = 0; y < bs.y; ++y) {
				const Vector3 local(origin.x + x * stride, origin.y + y * stride, origin.z + z * stride);
				const Vector3 p = params.voxel_to_planet.xform(local);
				const double r = p.length();
				planet_r[i] = r;
				dirs[i] = p / r;
				++i;
			}
		}
	}
	params.field.eval_radius_batch(dirs.data(), sx.data(), sy.data(), sz.data(), noise.data(), radii.data(), count);

	i = 0;
	for (int z = 0; z < bs.z; ++z) {
		for (int x = 0; x < bs.x; ++x) {
			for (int y = 0; y < bs.y; ++y) {
				const float sdf = float((planet_r[i] - radii[i]) * params.inv_voxel_size);
				out_buffer.set_voxel_f(sdf, x, y, z, VoxelBuffer::CHANNEL_SDF);
				++i;
			}
		}
	}
	return result;
}

void VoxelGeneratorPlanet::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_field", "field"), &VoxelGeneratorPlanet::set_field);
	ClassDB::bind_method(D_METHOD("get_field"), &VoxelGeneratorPlanet::get_field);
	ClassDB::bind_method(D_METHOD("set_voxel_to_planet", "transform"), &VoxelGeneratorPlanet::set_voxel_to_planet);
	ClassDB::bind_method(D_METHOD("get_voxel_to_planet"), &VoxelGeneratorPlanet::get_voxel_to_planet);
	ClassDB::bind_method(D_METHOD("get_sdf_at", "voxel_position"), &VoxelGeneratorPlanet::get_sdf_at);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "field", PROPERTY_HINT_RESOURCE_TYPE, "PlanetField3D"), "set_field", "get_field");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "voxel_to_planet"), "set_voxel_to_planet", "get_voxel_to_planet");
}

} // namespace zylann::voxel
