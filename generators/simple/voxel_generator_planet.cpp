#include "voxel_generator_planet.h"
#include "../../util/containers/std_vector.h"

#include "core/math/planet_lattice.h"

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

	// Only the carves whose influence box meets this block.
	LocalVector<uint32_t> nearby;
	const Vector3 block_lo(origin.x, origin.y, origin.z);
	const Vector3 block_hi(origin.x + (bs.x - 1) * stride, origin.y + (bs.y - 1) * stride, origin.z + (bs.z - 1) * stride);
	for (uint32_t c = 0; c < params.carve_centres.size(); ++c) {
		const double reach = params.carve_radii[c] + CARVE_MARGIN;
		const Vector3 &cc = params.carve_centres[c];
		if (cc.x + reach >= block_lo.x && cc.x - reach <= block_hi.x && cc.y + reach >= block_lo.y &&
				cc.y - reach <= block_hi.y && cc.z + reach >= block_lo.z && cc.z - reach <= block_hi.z) {
			nearby.push_back(c);
		}
	}

	i = 0;
	for (int z = 0; z < bs.z; ++z) {
		for (int x = 0; x < bs.x; ++x) {
			for (int y = 0; y < bs.y; ++y) {
				double sdf = (planet_r[i] - radii[i]) * params.inv_voxel_size;
				if (!nearby.is_empty()) {
					const Vector3 local(origin.x + x * stride, origin.y + y * stride, origin.z + z * stride);
					for (const uint32_t c : nearby) {
						const Vector3 d = local - params.carve_centres[c];
						const double reach = params.carve_radii[c] + CARVE_MARGIN;
						if (Math::abs(d.x) <= reach && Math::abs(d.y) <= reach && Math::abs(d.z) <= reach) {
							sdf = MAX(sdf, params.carve_radii[c] - d.length());
						}
					}
				}
				out_buffer.set_voxel_f(float(sdf), x, y, z, VoxelBuffer::CHANNEL_SDF);
				++i;
			}
		}
	}
	return result;
}

void VoxelGeneratorPlanet::add_carve(const Vector3 &p_centre, double p_radius) {
	RWLockWrite wlock(_parameters_lock);
	_parameters.carve_centres.push_back(p_centre);
	_parameters.carve_radii.push_back(p_radius);
}

void VoxelGeneratorPlanet::clear_carves() {
	RWLockWrite wlock(_parameters_lock);
	_parameters.carve_centres.clear();
	_parameters.carve_radii.clear();
}

int VoxelGeneratorPlanet::get_carve_count() const {
	RWLockRead rlock(_parameters_lock);
	return int(_parameters.carve_centres.size());
}

AABB VoxelGeneratorPlanet::get_carve_box(const Vector3 &p_centre, double p_radius) {
	const double reach = p_radius + CARVE_MARGIN;
	const Vector3 lo(Math::floor(p_centre.x - reach), Math::floor(p_centre.y - reach), Math::floor(p_centre.z - reach));
	const Vector3 hi(Math::ceil(p_centre.x + reach), Math::ceil(p_centre.y + reach), Math::ceil(p_centre.z + reach));
	return AABB(lo, hi - lo + Vector3(1, 1, 1));
}

Vector3i VoxelGeneratorPlanet::site_key_of(const Vector3 &p_point, int p_resolution) {
	int face = 0;
	double s = 0.0;
	double t = 0.0;
	PlanetLattice::direction_to_face(p_point.normalized(), face, s, t);
	const int res = MAX(p_resolution, 1);
	const int i = CLAMP(int(Math::floor((s + 1.0) * 0.5 * res)), 0, res - 1);
	const int j = CLAMP(int(Math::floor((t + 1.0) * 0.5 * res)), 0, res - 1);
	return Vector3i(face, i, j);
}

Transform3D VoxelGeneratorPlanet::make_site_transform(const Ref<PlanetField3D> &p_field, const Vector3i &p_key, int p_resolution,
		int p_voxels_across, int p_voxels_below) {
	ERR_FAIL_COND_V(p_field.is_null(), Transform3D());
	const double res = double(MAX(p_resolution, 1));
	auto dir_at = [&](double p_i, double p_j) -> Vector3 {
		const double s = PlanetLattice::face_warp((2.0 * p_i) / res - 1.0);
		const double t = PlanetLattice::face_warp((2.0 * p_j) / res - 1.0);
		return PlanetLattice::cube_point(p_key.x, s, t).normalized();
	};
	const double ci = double(p_key.y) + 0.5;
	const double cj = double(p_key.z) + 0.5;
	const Vector3 up = dir_at(ci, cj);
	// The cell's i direction, flattened into the tangent plane at its centre.
	const Vector3 across_i = dir_at(ci + 0.5, cj) - dir_at(ci - 0.5, cj);
	const Vector3 x_axis = (across_i - up * across_i.dot(up)).normalized();
	const Vector3 z_axis = x_axis.cross(up);
	// The cell's width in metres at the surface, averaged over both directions; the voxel edge follows.
	const double radius = double(p_field->eval_radius(up));
	const Vector3 across_j = dir_at(ci, cj + 0.5) - dir_at(ci, cj - 0.5);
	const double width = 0.5 * (across_i.length() + across_j.length()) * radius;
	const double voxel = width / double(MAX(p_voxels_across, 1));
	const Vector3 centre = up * radius;
	const double half = 0.5 * double(p_voxels_across) * voxel;
	const Vector3 origin = centre - up * (double(p_voxels_below) * voxel) - x_axis * half - z_axis * half;
	return Transform3D(Basis(x_axis * voxel, up * voxel, z_axis * voxel), origin);
}

void VoxelGeneratorPlanet::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_field", "field"), &VoxelGeneratorPlanet::set_field);
	ClassDB::bind_method(D_METHOD("get_field"), &VoxelGeneratorPlanet::get_field);
	ClassDB::bind_method(D_METHOD("set_voxel_to_planet", "transform"), &VoxelGeneratorPlanet::set_voxel_to_planet);
	ClassDB::bind_method(D_METHOD("get_voxel_to_planet"), &VoxelGeneratorPlanet::get_voxel_to_planet);
	ClassDB::bind_method(D_METHOD("get_sdf_at", "voxel_position"), &VoxelGeneratorPlanet::get_sdf_at);
	ClassDB::bind_static_method("VoxelGeneratorPlanet", D_METHOD("site_key_of", "point", "resolution"), &VoxelGeneratorPlanet::site_key_of);
	ClassDB::bind_method(D_METHOD("add_carve", "centre", "radius"), &VoxelGeneratorPlanet::add_carve);
	ClassDB::bind_method(D_METHOD("clear_carves"), &VoxelGeneratorPlanet::clear_carves);
	ClassDB::bind_method(D_METHOD("get_carve_count"), &VoxelGeneratorPlanet::get_carve_count);
	ClassDB::bind_static_method("VoxelGeneratorPlanet", D_METHOD("get_carve_box", "centre", "radius"), &VoxelGeneratorPlanet::get_carve_box);
	ClassDB::bind_static_method("VoxelGeneratorPlanet", D_METHOD("make_site_transform", "field", "key", "resolution", "voxels_across", "voxels_below"),
			&VoxelGeneratorPlanet::make_site_transform);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "field", PROPERTY_HINT_RESOURCE_TYPE, "PlanetField3D"), "set_field", "get_field");
	ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "voxel_to_planet"), "set_voxel_to_planet", "get_voxel_to_planet");
}

} // namespace zylann::voxel
