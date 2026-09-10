#include "test_planet_field_check.h"

#include "../../generators/graph/voxel_generator_graph.h"
#include "../../generators/graph/voxel_graph_function.h"
#include "../../storage/voxel_buffer.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/core/random_pcg.h"
#include "../../util/godot/core/string.h"
#include "../../util/io/log.h"
#include "../../util/math/vector3f.h"
#include "../../util/string/format.h"

#include "core/math/planet_hash_noise.h"
#include "core/math/planet_biome.h"
#include "core/math/planet_field.h"
#include "scene/resources/3d/planet_biome_3d.h"
#include "scene/resources/3d/planet_field_3d.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace zylann::voxel::tests {

using namespace pg;

namespace {

constexpr double PLANET_RADIUS_M = 3185500.0;
constexpr double PLANET_AMPLITUDE_M = 8000.0;
constexpr float NOISE_PERIOD_M = 2443.f;
constexpr int NOISE_OCTAVES = 13;
constexpr int NOISE_SEED = 1337;

int g_failures = 0;

void check(const bool condition, const char *what) {
	if (condition) {
		print_line(format("  PASS  {}", what));
	} else {
		print_line(format("  FAIL  {}", what));
		++g_failures;
	}
}

uint32_t float_bits(const float v) {
	uint32_t bits;
	std::memcpy(&bits, &v, sizeof(bits));
	return bits;
}

PlanetHashNoise::FbmParams make_fbm() {
	PlanetHashNoise::FbmParams p;
	p.seed = uint32_t(NOISE_SEED);
	p.period = NOISE_PERIOD_M;
	p.octaves = NOISE_OCTAVES;
	p.lacunarity = 2.f;
	p.gain = 0.5f;
	return p;
}

Ref<PlanetField3D> make_field() {
	Ref<PlanetField3D> field;
	field.instantiate();
	field->set_base_radius(PLANET_RADIUS_M);
	field->set_amplitude(PLANET_AMPLITUDE_M);
	field->set_seed(NOISE_SEED);
	field->set_period(NOISE_PERIOD_M);
	field->set_octaves(NOISE_OCTAVES);
	field->set_lacunarity(2.0);
	field->set_gain(0.5);
	return field;
}

// The same fbm as the field, expressed as a voxel graph so the generator path can be compared.
Ref<VoxelGeneratorGraph> make_graph() {
	Ref<VoxelGeneratorGraph> generator;
	generator.instantiate();
	Ref<VoxelGraphFunction> func = generator->get_main_function();
	if (func.is_null()) {
		return Ref<VoxelGeneratorGraph>();
	}
	VoxelGraphFunction &g = **func;

	const uint32_t n_x = g.create_node(VoxelGraphFunction::NODE_INPUT_X, Vector2());
	const uint32_t n_y = g.create_node(VoxelGraphFunction::NODE_INPUT_Y, Vector2());
	const uint32_t n_z = g.create_node(VoxelGraphFunction::NODE_INPUT_Z, Vector2());
	const uint32_t n_out = g.create_node(VoxelGraphFunction::NODE_OUTPUT_SDF, Vector2());
	const uint32_t n_noise = g.create_node(VoxelGraphFunction::NODE_PLANET_NOISE_3D, Vector2());

	g.set_node_param(n_noise, 0, NOISE_SEED);
	g.set_node_param(n_noise, 1, NOISE_PERIOD_M);
	g.set_node_param(n_noise, 2, NOISE_OCTAVES);
	g.set_node_param(n_noise, 3, 2.0f);
	g.set_node_param(n_noise, 4, 0.5f);

	g.add_connection(n_x, 0, n_noise, 0);
	g.add_connection(n_y, 0, n_noise, 1);
	g.add_connection(n_z, 0, n_noise, 2);
	g.add_connection(n_noise, 0, n_out, 0);

	const pg::CompilationResult result = generator->compile(false);
	if (!result.success) {
		print_line(format("  graph compile failed: {}", zylann::godot::to_std_string(result.message)));
		return Ref<VoxelGeneratorGraph>();
	}
	return generator;
}

Vector3 random_unit_dir(RandomPCG &rng) {
	// Uniform on the sphere: z uniform in [-1,1], azimuth uniform.
	const double z = double(rng.randf()) * 2.0 - 1.0;
	const double a = double(rng.randf()) * Math::TAU;
	const double r = Math::sqrt(MAX(0.0, 1.0 - z * z));
	return Vector3(r * Math::cos(a), r * Math::sin(a), z);
}

void check_scalar_vs_batched_noise() {
	const PlanetHashNoise::FbmParams p = make_fbm();
	const uint32_t count = 4096;

	StdVector<float> x(count), y(count), z(count), batched(count);
	RandomPCG rng(12345);
	for (uint32_t i = 0; i < count; ++i) {
		const Vector3 d = random_unit_dir(rng);
		x[i] = float(d.x * PLANET_RADIUS_M);
		y[i] = float(d.y * PLANET_RADIUS_M);
		z[i] = float(d.z * PLANET_RADIUS_M);
	}

	PlanetHashNoise::fbm_3d_series(x.data(), y.data(), z.data(), batched.data(), count, p);

	uint32_t mismatches = 0;
	for (uint32_t i = 0; i < count; ++i) {
		const float scalar = PlanetHashNoise::fbm_3d(x[i], y[i], z[i], p);
		if (float_bits(scalar) != float_bits(batched[i])) {
			++mismatches;
		}
	}
	check(mismatches == 0, "scalar fbm_3d == batched fbm_3d_series, bit for bit");
	if (mismatches != 0) {
		print_line(format("        {} of {} samples differ", mismatches, count));
	}
}

void check_field_scalar_vs_batched() {
	Ref<PlanetField3D> field = make_field();
	const uint32_t count = 4096;

	StdVector<Vector3> dirs(count);
	StdVector<double> batched(count);
	RandomPCG rng(9876);
	for (uint32_t i = 0; i < count; ++i) {
		dirs[i] = random_unit_dir(rng);
	}

	StdVector<double> sx(count), sy(count), sz(count);
	StdVector<float> noise(count);
	field->get_data().eval_radius_batch(
			dirs.data(), sx.data(), sy.data(), sz.data(), noise.data(), batched.data(), count);

	uint32_t mismatches = 0;
	double worst = 0.0;
	for (uint32_t i = 0; i < count; ++i) {
		const double scalar = field->eval_radius(dirs[i]);
		if (scalar != batched[i]) {
			++mismatches;
			worst = MAX(worst, Math::abs(scalar - batched[i]));
		}
	}
	check(mismatches == 0, "PlanetField3D eval_radius == eval_radius_batch, bit for bit");
	if (mismatches != 0) {
		print_line(format("        {} of {} samples differ, worst {} m", mismatches, count, worst));
	}
}

void check_graph_matches_noise() {
	Ref<VoxelGeneratorGraph> generator = make_graph();
	if (generator.is_null()) {
		check(false, "voxel graph with PlanetNoise3D compiles");
		return;
	}
	check(true, "voxel graph with PlanetNoise3D compiles");

	const PlanetHashNoise::FbmParams p = make_fbm();
	const uint32_t count = 2048;

	StdVector<float> x(count), y(count), z(count), graph_out(count);
	RandomPCG rng(4242);
	for (uint32_t i = 0; i < count; ++i) {
		const Vector3 d = random_unit_dir(rng);
		x[i] = float(d.x * PLANET_RADIUS_M);
		y[i] = float(d.y * PLANET_RADIUS_M);
		z[i] = float(d.z * PLANET_RADIUS_M);
	}

	generator->generate_series(
			to_span_const(x),
			to_span_const(y),
			to_span_const(z),
			VoxelBuffer::CHANNEL_SDF,
			to_span(graph_out),
			Vector3f(-1e9f, -1e9f, -1e9f),
			Vector3f(1e9f, 1e9f, 1e9f)
	);

	uint32_t mismatches = 0;
	for (uint32_t i = 0; i < count; ++i) {
		const float direct = PlanetHashNoise::fbm_3d(x[i], y[i], z[i], p);
		if (float_bits(direct) != float_bits(graph_out[i])) {
			++mismatches;
		}
	}
	check(mismatches == 0, "voxel graph PlanetNoise3D == PlanetHashNoise::fbm_3d, bit for bit");
	if (mismatches != 0) {
		print_line(format("        {} of {} samples differ", mismatches, count));
	}
}

void check_global_bounds() {
	Ref<PlanetField3D> field = make_field();
	const double lo = field->get_min_radius();
	const double hi = field->get_max_radius();

	const uint32_t count = 1000000;
	RandomPCG rng(777);
	uint32_t violations = 0;
	double observed_lo = INFINITY;
	double observed_hi = -INFINITY;

	for (uint32_t i = 0; i < count; ++i) {
		const Vector3 d = random_unit_dir(rng);
		const double r = field->eval_radius(d);
		observed_lo = MIN(observed_lo, r);
		observed_hi = MAX(observed_hi, r);
		if (r < lo || r > hi) {
			++violations;
		}
	}

	check(violations == 0, "global radius bounds contain f over 1M directions");
	print_line(format("        bounds [{}, {}]  observed [{}, {}]", lo, hi, observed_lo, observed_hi));
	print_line(format("        bound is {}x wider than the observed range",
			(hi - lo) / MAX(1e-9, observed_hi - observed_lo)));
}

void check_cone_bounds() {
	Ref<PlanetField3D> field = make_field();

	RandomPCG rng(31337);
	uint32_t violations = 0;
	double worst_overshoot = 0.0;
	double tightness_sum = 0.0;
	const int cone_count = 200;
	const int samples_per_cone = 400;

	for (int c = 0; c < cone_count; ++c) {
		const Vector3 axis = random_unit_dir(rng);
		// Cone half-angles spanning a body-sized patch up to the wide-cone fallback threshold.
		const double half_angle = 0.00002 + double(rng.randf()) * 0.05;

		double lo = 0.0;
		double hi = 0.0;
		field->get_data().get_bounds_for_cone(axis, half_angle, lo, hi);

		Vector3 u = axis.cross(Vector3(0, 0, 1));
		if (u.length_squared() < 1e-6) {
			u = axis.cross(Vector3(0, 1, 0));
		}
		u.normalize();
		const Vector3 v = axis.cross(u).normalized();
		const double tan_half = Math::tan(half_angle);

		double observed_lo = INFINITY;
		double observed_hi = -INFINITY;
		for (int s = 0; s < samples_per_cone; ++s) {
			// Uniform over the disc, so the interior is sampled rather than just the rim.
			const double rr = Math::sqrt(double(rng.randf())) * tan_half;
			const double a = double(rng.randf()) * Math::TAU;
			const Vector3 dir = (axis + u * (rr * Math::cos(a)) + v * (rr * Math::sin(a))).normalized();
			const double r = field->eval_radius(dir);
			observed_lo = MIN(observed_lo, r);
			observed_hi = MAX(observed_hi, r);
			if (r < lo || r > hi) {
				++violations;
				worst_overshoot = MAX(worst_overshoot, MAX(lo - r, r - hi));
			}
		}
		tightness_sum += (hi - lo) / MAX(1e-9, observed_hi - observed_lo);
	}

	check(violations == 0, "cone bounds contain f over 200 cones x 400 interior samples");
	if (violations != 0) {
		print_line(format("        {} violations, worst overshoot {} m", violations, worst_overshoot));
	}
	print_line(format("        cone bound averages {}x wider than the sampled range",
			tightness_sum / double(cone_count)));
}

void check_lipschitz_constant() {
	// The cone bounds add a margin of Lipschitz * distance-to-nearest-sample. If the real gradient
	// can exceed the assumed constant, the margin is too small and the bound stops containing f,
	// which silently drops collisions. Re-measure it here rather than trusting the comment.
	const uint32_t count = 400000;
	const float h = 1e-3f;
	RandomPCG rng(2024);
	double max_grad = 0.0;

	for (uint32_t i = 0; i < count; ++i) {
		// Lattice-space coordinates, so the measured constant is per lattice unit.
		const float px = (float(rng.randf()) - 0.5f) * 512.f;
		const float py = (float(rng.randf()) - 0.5f) * 512.f;
		const float pz = (float(rng.randf()) - 0.5f) * 512.f;

		const float gx = (PlanetHashNoise::perlin_3d(px + h, py, pz, 5u) - PlanetHashNoise::perlin_3d(px - h, py, pz, 5u)) / (2.f * h);
		const float gy = (PlanetHashNoise::perlin_3d(px, py + h, pz, 5u) - PlanetHashNoise::perlin_3d(px, py - h, pz, 5u)) / (2.f * h);
		const float gz = (PlanetHashNoise::perlin_3d(px, py, pz + h, 5u) - PlanetHashNoise::perlin_3d(px, py, pz - h, 5u)) / (2.f * h);

		max_grad = MAX(max_grad, Math::sqrt(double(gx) * gx + double(gy) * gy + double(gz) * gz));
	}

	const double assumed = PlanetFieldData::PERLIN_3D_LIPSCHITZ_PER_LATTICE_UNIT;
	check(max_grad < assumed, "measured Perlin gradient stays under the assumed Lipschitz constant");
	print_line(format("        measured max |grad| {} per lattice unit, assumed {} ({}x headroom)",
			max_grad, assumed, assumed / MAX(1e-9, max_grad)));
}


// ---------------------------------------------------------------- Phase A: shaping and derivatives

// The shaped evaluator must reduce EXACTLY to the old one when the new features are all off. If it
// does not, then simply adopting the new code path silently moves every planet's surface, and every
// stored world with it.
void check_shaped_reduces_to_plain() {
	const PlanetHashNoise::FbmParams fbm = make_fbm();
	const PlanetHashNoise::ShapeMix shape; // {1, 0, 0}
	const PlanetHashNoise::WarpParams warp; // amplitude 0 -> disabled
	RandomPCG rng(4711);
	uint32_t mismatches = 0;
	const uint32_t count = 20000;

	for (uint32_t i = 0; i < count; ++i) {
		const double x = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
		const double y = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
		const double z = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;

		const float plain = PlanetHashNoise::fbm_3d_precise(x, y, z, fbm);
		const float shaped = PlanetHashNoise::fbm_3d_shaped_precise(x, y, z, fbm, shape, warp, 0.f, 1.f);
		if (float_bits(plain) != float_bits(shaped)) {
			++mismatches;
		}
	}
	check(mismatches == 0, "shaped fBm is bit-identical to plain fBm when shaping is neutral");
	print_line(format("        {} of {} samples differ", mismatches, count));
}

// The value returned alongside the gradient must be the same value the plain evaluator returns, or
// the derivative describes a different surface than the one being collided against.
void check_deriv_value_matches() {
	RandomPCG rng(99);
	uint32_t mismatches = 0;
	const uint32_t count = 50000;
	for (uint32_t i = 0; i < count; ++i) {
		const double x = (double(rng.randf()) - 0.5) * 512.0;
		const double y = (double(rng.randf()) - 0.5) * 512.0;
		const double z = (double(rng.randf()) - 0.5) * 512.0;
		float dx = 0.f;
		float dy = 0.f;
		float dz = 0.f;
		const float a = PlanetHashNoise::perlin_3d_precise(x, y, z, 7u);
		const float b = PlanetHashNoise::perlin_3d_precise_deriv(x, y, z, 7u, dx, dy, dz);
		if (float_bits(a) != float_bits(b)) {
			++mismatches;
		}
	}
	check(mismatches == 0, "perlin_3d_precise_deriv value is bit-identical to perlin_3d_precise");
	print_line(format("        {} of {} samples differ", mismatches, count));
}

// The analytic gradient must agree with finite differences. A wrong gradient would not crash
// anything -- it would just make the slope weighting attenuate the wrong octaves, which is
// impossible to spot by eye.
void check_analytic_derivative() {
	RandomPCG rng(1234);
	const double h = 1e-4;
	double worst_rel = 0.0;
	double worst_abs = 0.0;
	const uint32_t count = 20000;

	for (uint32_t i = 0; i < count; ++i) {
		const double x = (double(rng.randf()) - 0.5) * 256.0;
		const double y = (double(rng.randf()) - 0.5) * 256.0;
		const double z = (double(rng.randf()) - 0.5) * 256.0;

		float ax = 0.f;
		float ay = 0.f;
		float az = 0.f;
		PlanetHashNoise::perlin_3d_precise_deriv(x, y, z, 3u, ax, ay, az);

		const double fx = (double(PlanetHashNoise::perlin_3d_precise(x + h, y, z, 3u)) -
								  double(PlanetHashNoise::perlin_3d_precise(x - h, y, z, 3u))) / (2.0 * h);
		const double fy = (double(PlanetHashNoise::perlin_3d_precise(x, y + h, z, 3u)) -
								  double(PlanetHashNoise::perlin_3d_precise(x, y - h, z, 3u))) / (2.0 * h);
		const double fz = (double(PlanetHashNoise::perlin_3d_precise(x, y, z + h, 3u)) -
								  double(PlanetHashNoise::perlin_3d_precise(x, y, z - h, 3u))) / (2.0 * h);

		const double ex = Math::abs(double(ax) - fx);
		const double ey = Math::abs(double(ay) - fy);
		const double ez = Math::abs(double(az) - fz);
		const double e = MAX(ex, MAX(ey, ez));
		worst_abs = MAX(worst_abs, e);
		const double mag = MAX(1.0, Math::sqrt(fx * fx + fy * fy + fz * fz));
		worst_rel = MAX(worst_rel, e / mag);
	}
	// Central differences on a float-valued function carry their own error, so this is a loose bound
	// on purpose; a wrong derivative misses by order 1, not by 1e-2.
	check(worst_rel < 0.02, "analytic Perlin gradient matches central differences");
	print_line(format("        worst absolute {} , worst relative {}", worst_abs, worst_rel));
}

// ShapeMix::bounds() is used as an EXACT global bound on the surface, so a single sample outside it
// is a correctness failure, not a tolerance question.
void check_shape_bounds_exact() {
	const PlanetHashNoise::FbmParams fbm = make_fbm();
	const PlanetHashNoise::WarpParams warp;
	RandomPCG rng(555);

	struct Case {
		PlanetHashNoise::ShapeMix mix;
		const char *name;
	};
	PlanetHashNoise::ShapeMix m_std;
	PlanetHashNoise::ShapeMix m_bil;
	m_bil.standard = 0.f;
	m_bil.billow = 1.f;
	PlanetHashNoise::ShapeMix m_rid;
	m_rid.standard = 0.f;
	m_rid.ridged = 1.f;
	PlanetHashNoise::ShapeMix m_mix;
	m_mix.standard = 0.34f;
	m_mix.billow = 0.33f;
	m_mix.ridged = 0.33f;

	const Case cases[4] = { { m_std, "standard" }, { m_bil, "billow" }, { m_rid, "ridged" }, { m_mix, "mixed" } };

	uint32_t violations = 0;
	for (int c = 0; c < 4; ++c) {
		float lo = 0.f;
		float hi = 0.f;
		cases[c].mix.bounds(lo, hi);
		for (uint32_t i = 0; i < 20000; ++i) {
			const double x = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
			const double y = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
			const double z = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
			const float v = PlanetHashNoise::fbm_3d_shaped_precise(x, y, z, fbm, cases[c].mix, warp, 0.f, 1.f);
			if (v < lo - 1e-5f || v > hi + 1e-5f) {
				++violations;
			}
		}
	}
	check(violations == 0, "shaped fBm stays inside ShapeMix::bounds() for every shape mix");
	print_line(format("        {} violations over 80000 samples", violations));
}

// Slope weighting must not push the field outside its declared range either, since the global bounds
// feed the broadphase.
void check_slope_erosion_bounded() {
	const PlanetHashNoise::FbmParams fbm = make_fbm();
	const PlanetHashNoise::ShapeMix shape;
	const PlanetHashNoise::WarpParams warp;
	RandomPCG rng(31337);
	uint32_t violations = 0;
	double worst = 0.0;
	for (uint32_t i = 0; i < 40000; ++i) {
		const double x = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
		const double y = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
		const double z = (double(rng.randf()) - 0.5) * 2.0 * PLANET_RADIUS_M;
		const float v = PlanetHashNoise::fbm_3d_shaped_precise(x, y, z, fbm, shape, warp, 2.0f, float(PLANET_AMPLITUDE_M));
		worst = MAX(worst, Math::abs(double(v)));
		if (v < -1.f - 1e-5f || v > 1.f + 1e-5f) {
			++violations;
		}
	}
	check(violations == 0, "slope-weighted fBm stays within [-1, 1]");
	print_line(format("        {} violations, worst magnitude {}", violations, worst));
}

// The bedrock floor has to be enforced by the same code the collider uses, not bolted onto the
// renderer, or the two disagree about where the ground is.
void check_floor_clamp() {
	PlanetFieldData data;
	data.base_radius = PLANET_RADIUS_M;
	data.amplitude = PLANET_AMPLITUDE_M;
	data.fbm = make_fbm();
	data.floor_radius = PLANET_RADIUS_M; // Clamp away the entire lower half of the range.

	RandomPCG rng(808);
	uint32_t below = 0;
	uint32_t clamped = 0;
	for (uint32_t i = 0; i < 20000; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		const double r = data.eval_radius(d);
		if (r < data.floor_radius - 1e-9) {
			++below;
		}
		if (r <= data.floor_radius + 1e-9) {
			++clamped;
		}
	}
	check(below == 0, "eval_radius never returns a radius below floor_radius");
	check(clamped > 0, "floor_radius actually clamps part of the surface (test is not vacuous)");
	check(data.get_min_radius() >= data.floor_radius, "get_min_radius respects floor_radius");
	print_line(format("        {} samples below floor, {} clamped to it", below, clamped));

	PlanetFieldData dry = data;
	dry.floor_radius = PLANET_RADIUS_M;
	dry.sea_level_radius = PLANET_RADIUS_M - 100.0;
	check(!dry.has_water(), "sea level below bedrock means the planet is dry");
	dry.sea_level_radius = PLANET_RADIUS_M + 100.0;
	check(dry.has_water(), "sea level above bedrock means the planet has water");
}

void check_smooth_terracing() {
	PlanetFieldData data;
	data.terrace_step = 100.0;
	check(Math::is_equal_approx(data.apply_terracing(12.5, 1.f), 0.0),
			"full terracing creates a broad flat lower shelf");
	check(Math::is_equal_approx(data.apply_terracing(40.0, 1.f), 0.0),
			"the lower shelf remains flat through forty percent of the step");
	check(data.apply_terracing(41.0, 1.f) > 0.0,
			"the narrow terrace transition begins immediately above forty percent");
	check(Math::is_equal_approx(data.apply_terracing(50.0, 1.f), 50.0),
			"the cubic terrace transition crosses the original height at mid-step");
	check(Math::is_equal_approx(data.apply_terracing(60.0, 1.f), 100.0),
			"the upper shelf begins by sixty percent of the step");
	check(Math::is_equal_approx(data.apply_terracing(87.5, 1.f), 100.0),
			"full terracing creates a broad flat upper shelf");
	check(Math::is_equal_approx(data.apply_terracing(12.5, 0.5f), 6.25),
			"partial terrace strength blends continuously with the original field");
	check(Math::is_equal_approx(data.apply_terracing(-12.5, 1.f), 0.0),
			"terrace levels remain symmetric below the base radius");
}

// Same contract as check_field_scalar_vs_batched, but with every Phase A feature switched on at once.
// The collider uses the batched path and the renderer uses the scalar one; if they diverge, the
// ground you see is not the ground you stand on.
void check_shaped_scalar_vs_batched() {
	PlanetFieldData data;
	data.base_radius = PLANET_RADIUS_M;
	data.amplitude = PLANET_AMPLITUDE_M;
	data.fbm = make_fbm();
	data.shape.standard = 0.3f;
	data.shape.billow = 0.2f;
	data.shape.ridged = 0.5f;
	data.warp.amplitude = 600.f;
	data.warp.period = 40000.f;
	data.slope_erosion = 1.5;
	data.floor_radius = PLANET_RADIUS_M - 4000.0;

	const uint32_t count = 512;
	StdVector<Vector3> dirs(count);
	StdVector<double> sx(count);
	StdVector<double> sy(count);
	StdVector<double> sz(count);
	StdVector<float> noise(count);
	StdVector<double> radius(count);

	RandomPCG rng(2718);
	for (uint32_t i = 0; i < count; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			d = Vector3(0, 1, 0);
		}
		dirs[i] = d.normalized();
	}

	data.eval_radius_batch(dirs.data(), sx.data(), sy.data(), sz.data(), noise.data(), radius.data(), count);

	uint32_t mismatches = 0;
	double worst = 0.0;
	for (uint32_t i = 0; i < count; ++i) {
		const double scalar = data.eval_radius(dirs[i]);
		if (scalar != radius[i]) {
			++mismatches;
			worst = MAX(worst, Math::abs(scalar - radius[i]));
		}
	}
	check(mismatches == 0, "shaped scalar and batched radii agree exactly with all features enabled");
	print_line(format("        {} of {} differ, worst {} m", mismatches, count, worst));
}

// The Lipschitz bound has to hold for the field as actually configured, warp and shaping included.
// This is the check that stands between a mis-set bound and silently missed collisions.
void check_lipschitz_with_features() {
	PlanetFieldData data;
	data.base_radius = PLANET_RADIUS_M;
	data.amplitude = PLANET_AMPLITUDE_M;
	data.fbm = make_fbm();
	data.shape.standard = 0.2f;
	data.shape.billow = 0.3f;
	data.shape.ridged = 0.5f;
	data.warp.amplitude = 400.f;
	data.warp.period = 30000.f;
	data.slope_erosion = 1.0;

	// Which feature inflates the slope? Measured one at a time, because a single combined number
	// says the bound is wrong without saying why.
	for (int cfg = 0; cfg < 4; ++cfg) {
		PlanetFieldData d2 = data;
		d2.shape = PlanetHashNoise::ShapeMix();
		d2.warp = PlanetHashNoise::WarpParams();
		d2.slope_erosion = 0.0;
		const char *label = "plain";
		if (cfg == 1) {
			d2.shape = data.shape;
			label = "shape only";
		} else if (cfg == 2) {
			d2.warp = data.warp;
			label = "warp only";
		} else if (cfg == 3) {
			d2.slope_erosion = data.slope_erosion;
			label = "erosion only";
		}
		RandomPCG r2(6161);
		double ms = 0.0;
		for (uint32_t i = 0; i < 20000; ++i) {
			Vector3 dd(double(r2.randf()) - 0.5, double(r2.randf()) - 0.5, double(r2.randf()) - 0.5);
			if (dd.length_squared() < 1e-12) {
				continue;
			}
			dd.normalize();
			Vector3 tt = dd.cross(Vector3(0, 0, 1));
			if (tt.length_squared() < 1e-9) {
				tt = dd.cross(Vector3(0, 1, 0));
			}
			tt.normalize();
			const Vector3 db = (dd * PLANET_RADIUS_M + tt * 0.05).normalized();
			ms = MAX(ms, Math::abs(d2.eval_radius(db) - d2.eval_radius(dd)) / 0.05);
		}
		print_line(format("        [{}] max slope {} , bound {}", label, ms, d2.get_lipschitz_per_metre()));
	}

	const double bound = data.get_lipschitz_per_metre();
	const double step = 0.05; // metres along the surface
	RandomPCG rng(6161);
	double max_slope = 0.0;

	for (uint32_t i = 0; i < 40000; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		// A tangent direction, to step along the surface rather than through it.
		Vector3 t = d.cross(Vector3(0, 0, 1));
		if (t.length_squared() < 1e-9) {
			t = d.cross(Vector3(0, 1, 0));
		}
		t.normalize();
		const Vector3 d2 = (d * PLANET_RADIUS_M + t * step).normalized();
		const double r1 = data.eval_radius(d);
		const double r2 = data.eval_radius(d2);
		max_slope = MAX(max_slope, Math::abs(r2 - r1) / step);
	}

	check(max_slope < bound, "measured surface slope stays under get_lipschitz_per_metre()");
	print_line(format("        measured max slope {} per metre, bound {} ({}x headroom)",
			max_slope, bound, bound / MAX(1e-9, max_slope)));
}


// ---------------------------------------------------------------- Phase B: macro maps and biomes

PlanetMacroMapRef make_macro_map(const int p_resolution) {
	std::shared_ptr<PlanetMacroMapData> m = std::make_shared<PlanetMacroMapData>();
	m->allocate(p_resolution);
	PlanetMacroBakeParams params;
	params.seed = 2024u;
	params.continent_period = 6000000.f;
	params.moisture_period = 3000000.f;
	m->bake(params, PLANET_RADIUS_M);
	return m;
}

// Face selection and the inverse mapping must agree, or a baked texel ends up describing a different
// direction than the one that later reads it -- climate would be subtly rotated per face.
void check_cubemap_roundtrip() {
	RandomPCG rng(77);
	double worst = 0.0;
	uint32_t face_mismatch = 0;
	for (uint32_t i = 0; i < 50000; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		int face = 0;
		double u = 0.0;
		double v = 0.0;
		PlanetMacroMapData::direction_to_face_uv(d, face, u, v);
		if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) {
			++face_mismatch;
			continue;
		}
		const Vector3 back = PlanetMacroMapData::face_uv_to_direction(face, u, v);
		worst = MAX(worst, (back - d).length());
	}
	check(face_mismatch == 0, "cubemap face selection always produces uv inside [0, 1]");
	check(worst < 1e-9, "cubemap direction -> (face, uv) -> direction round-trips exactly");
	print_line(format("        worst round-trip error {}", worst));
}

// A seam that jumps is a visible ring around the planet. Sampling either side of a cube edge must
// give the same climate.
void check_macro_map_seam_continuity() {
	const PlanetMacroMapRef map = make_macro_map(256);
	RandomPCG rng(1010);
	double worst = 0.0;
	const double eps = 1e-7;

	for (uint32_t i = 0; i < 20000; ++i) {
		// Directions right on a cube edge: one component at its maximum, another equal to it.
		const double t = double(rng.randf()) * 2.0 - 1.0;
		Vector3 d;
		switch (i % 3) {
			case 0:
				d = Vector3(1.0, 1.0, t);
				break;
			case 1:
				d = Vector3(1.0, t, 1.0);
				break;
			default:
				d = Vector3(t, 1.0, 1.0);
				break;
		}
		d.normalize();
		Vector3 off = Vector3(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (off.length_squared() < 1e-12) {
			continue;
		}
		off.normalize();
		const Vector3 a = (d + off * eps).normalized();
		const Vector3 b = (d - off * eps).normalized();
		const PlanetMacroSample sa = map->sample(a);
		const PlanetMacroSample sb = map->sample(b);
		worst = MAX(worst, double(Math::abs(sa.height - sb.height)));
		worst = MAX(worst, double(Math::abs(sa.temperature - sb.temperature)));
		worst = MAX(worst, double(Math::abs(sa.moisture - sb.moisture)));
	}
	// Edge texels clamp rather than reaching into the neighbour, so a sub-texel flattening is
	// expected; a real seam would show up as a step of order the channel range.
	check(worst < 0.02, "macro map is continuous across cube face seams");
	print_line(format("        worst discontinuity across a seam {}", worst));
}

void check_macro_map_channels_in_range() {
	const PlanetMacroMapRef map = make_macro_map(128);
	RandomPCG rng(4242);
	uint32_t out_of_range = 0;
	double h_lo = 1e9;
	double h_hi = -1e9;
	for (uint32_t i = 0; i < 50000; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		const PlanetMacroSample sm = map->sample(d);
		if (sm.height < -1e-4f || sm.height > 1.0001f || sm.temperature < -1e-4f || sm.temperature > 1.0001f ||
				sm.moisture < -1e-4f || sm.moisture > 1.0001f || sm.weirdness < -1e-4f || sm.weirdness > 1.0001f) {
			++out_of_range;
		}
		h_lo = MIN(h_lo, double(sm.height));
		h_hi = MAX(h_hi, double(sm.height));
	}
	check(out_of_range == 0, "all macro map channels stay within [0, 1]");
	check(h_hi - h_lo > 0.2, "macro height channel actually varies (bake is not flat)");
	print_line(format("        {} out of range, height spans [{}, {}]", out_of_range, h_lo, h_hi));
}

// Blending has to be continuous in climate space. The nearest-N scheme this replaced steps whenever
// the N-th nearest anchor changes, which would put a visible ridge along every biome border.
void check_global_biome_frequency() {
	PlanetBiomeTable table = PlanetBiomeTable::make_default();
	for (int i = 0; i < table.count; ++i) {
		table.entries[i].frequency_scale = 1.f;
	}
	uint32_t changed = 0;
	for (int i = 0; i < 10000; ++i) {
		PlanetBlendedParams b;
		table.blend(0.48f + float(i) * 1e-7f, 0.61f + float(i) * 2e-8f, b);
		changed += b.frequency_scale != 1.f;
	}
	check(changed == 0, "global biome frequency stays exactly constant at adjacent climate samples");
}

void check_biome_blend_continuity() {
	const PlanetBiomeTable table = PlanetBiomeTable::make_default();
	const int steps = 20000;
	double worst_amp = 0.0;
	double worst_freq = 0.0;
	double worst_shape = 0.0;
	PlanetBlendedParams prev;
	// Must start where the sweep starts, not at the origin, or the first comparison spans the whole
	// climate square and reports a discontinuity that is not there.
	table.blend(0.f, 1.f, prev);

	for (int i = 1; i <= steps; ++i) {
		// A diagonal sweep across the whole climate square, so every anchor is passed.
		const float t = float(i) / float(steps);
		PlanetBlendedParams cur;
		table.blend(t, 1.f - t, cur);
		worst_amp = MAX(worst_amp, double(Math::abs(cur.amplitude_scale - prev.amplitude_scale)));
		worst_freq = MAX(worst_freq, double(Math::abs(cur.frequency_scale - prev.frequency_scale)));
		worst_shape = MAX(worst_shape, double(Math::abs(cur.shape.ridged - prev.shape.ridged)));
		prev = cur;
	}
	// Per-step change over a 20000-step sweep; a discontinuity would be orders of magnitude larger.
	check(worst_amp < 0.01 && worst_freq < 0.01 && worst_shape < 0.01,
			"biome blending is continuous across the whole climate square");
	print_line(format("        worst per-step jump: amp {} freq {} ridged {}", worst_amp, worst_freq, worst_shape));
}

void check_biome_weights_normalized() {
	const PlanetBiomeTable table = PlanetBiomeTable::make_default();
	RandomPCG rng(313);
	uint32_t out_of_hull = 0;
	float amp_lo = 1e9f;
	float amp_hi = -1e9f;
	float min_amp = 1e9f;
	float max_amp = -1e9f;
	for (int i = 0; i < table.count; ++i) {
		min_amp = MIN(min_amp, table.entries[i].amplitude_scale);
		max_amp = MAX(max_amp, table.entries[i].amplitude_scale);
	}
	for (uint32_t i = 0; i < 50000; ++i) {
		const float t = float(rng.randf());
		const float m = float(rng.randf());
		PlanetBlendedParams b;
		table.blend(t, m, b);
		// A normalized convex blend can never leave the hull of what it blends.
		if (b.amplitude_scale < min_amp - 1e-4f || b.amplitude_scale > max_amp + 1e-4f) {
			++out_of_hull;
		}
		const float shape_sum = b.shape.standard + b.shape.billow + b.shape.ridged;
		if (Math::abs(shape_sum - 1.f) > 1e-3f) {
			++out_of_hull;
		}
		amp_lo = MIN(amp_lo, b.amplitude_scale);
		amp_hi = MAX(amp_hi, b.amplitude_scale);
	}
	check(out_of_hull == 0, "blended parameters stay inside the biome table's hull and shape weights sum to 1");
	print_line(format("        blended amplitude spans [{}, {}], table spans [{}, {}]", amp_lo, amp_hi, min_amp, max_amp));
}

PlanetFieldData make_biome_field() {
	PlanetFieldData data;
	data.base_radius = PLANET_RADIUS_M;
	data.amplitude = PLANET_AMPLITUDE_M;
	data.fbm = make_fbm();
	data.macro_map = make_macro_map(128);
	data.biomes = PlanetBiomeTable::make_default();
	data.macro_height_scale = 12000.0;
	data.mountain_amplitude = 1.5;
	data.mountain_ridge = 0.8;
	data.terrace_step = 100.0;
	for (int i = 0; i < data.biomes.count; ++i) {
		data.biomes.entries[i].terrace_strength = 0.65f;
	}
	data.warp.amplitude = 300.f;
	data.warp.period = 40000.f;
	data.slope_erosion = 1.0;
	return data;
}

// With biomes the amplitude, base elevation and shape all vary per point, so the exact global bounds
// have to be taken over the whole table. These bounds feed the broadphase: a violation is a body that
// never gets tested against the ground.
void check_biome_global_bounds() {
	const PlanetFieldData data = make_biome_field();
	const double lo = data.get_min_radius();
	const double hi = data.get_max_radius();

	RandomPCG rng(9090);
	uint32_t violations = 0;
	double obs_lo = 1e30;
	double obs_hi = -1e30;
	for (uint32_t i = 0; i < 200000; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		const double r = data.eval_radius(d);
		if (r < lo - 1e-6 || r > hi + 1e-6) {
			++violations;
		}
		obs_lo = MIN(obs_lo, r);
		obs_hi = MAX(obs_hi, r);
	}
	check(violations == 0, "global bounds contain f with biomes, macro height and mountains enabled");
	print_line(format("        bounds [{}, {}]  observed [{}, {}]", lo, hi, obs_lo, obs_hi));
	print_line(format("        bound is {}x wider than observed", (hi - lo) / MAX(1e-9, obs_hi - obs_lo)));
}

void check_biome_scalar_vs_batched() {
	const PlanetFieldData data = make_biome_field();
	const uint32_t count = 512;
	StdVector<Vector3> dirs(count);
	StdVector<double> sx(count);
	StdVector<double> sy(count);
	StdVector<double> sz(count);
	StdVector<float> noise(count);
	StdVector<double> radius(count);

	RandomPCG rng(1123);
	for (uint32_t i = 0; i < count; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			d = Vector3(0, 1, 0);
		}
		dirs[i] = d.normalized();
	}
	data.eval_radius_batch(dirs.data(), sx.data(), sy.data(), sz.data(), noise.data(), radius.data(), count);

	uint32_t mismatches = 0;
	for (uint32_t i = 0; i < count; ++i) {
		if (data.eval_radius(dirs[i]) != radius[i]) {
			++mismatches;
		}
	}
	check(mismatches == 0, "scalar and batched radii agree exactly with biomes enabled");
	print_line(format("        {} of {} differ", mismatches, count));
}

void check_biome_lipschitz() {
	const PlanetFieldData data = make_biome_field();
	const double bound = data.get_lipschitz_per_metre();
	const double step = 0.05;
	RandomPCG rng(5150);
	double max_slope = 0.0;
	for (uint32_t i = 0; i < 40000; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		Vector3 t = d.cross(Vector3(0, 0, 1));
		if (t.length_squared() < 1e-9) {
			t = d.cross(Vector3(0, 1, 0));
		}
		t.normalize();
		const Vector3 d2 = (d * PLANET_RADIUS_M + t * step).normalized();
		max_slope = MAX(max_slope, Math::abs(data.eval_radius(d2) - data.eval_radius(d)) / step);
	}
	check(max_slope < bound, "measured slope stays under the Lipschitz bound with biomes enabled");
	print_line(format("        measured max slope {} per metre, bound {} ({}x headroom)",
			max_slope, bound, bound / MAX(1e-9, max_slope)));
}

// Authored maps have to survive a round-trip through the image format, or "bake, export, paint,
// re-import" loses the edit it was made for.
void check_macro_map_image_roundtrip() {
	Ref<PlanetMacroMap3D> map;
	map.instantiate();
	map->set_resolution(64);
	map->set_bake_radius(PLANET_RADIUS_M);
	map->bake();
	check(map->is_baked(), "PlanetMacroMap3D bakes");

	const Vector3 probe = Vector3(0.4, 0.7, 0.2).normalized();
	const Color before = map->sample_direction(probe);

	Ref<PlanetMacroMap3D> copy;
	copy.instantiate();
	copy->set_resolution(64);
	bool ok = true;
	for (int face = 0; face < 6; ++face) {
		Ref<Image> img = map->get_face_image(face);
		if (img.is_null() || copy->set_face_image(face, img) != OK) {
			ok = false;
		}
	}
	check(ok, "every face exports to an Image and imports back");
	const Color after = copy->sample_direction(probe);
	const double err = MAX(MAX(Math::abs(before.r - after.r), Math::abs(before.g - after.g)),
			MAX(Math::abs(before.b - after.b), Math::abs(before.a - after.a)));
	// Half-float storage, so exact equality is not expected; 8-bit would have been ~4e-3.
	check(err < 1e-3, "macro map survives an image round-trip");
	print_line(format("        worst channel error after round-trip {}", err));
}

// Biomes must actually change the terrain, or every check above passes vacuously.
void check_biomes_change_the_surface() {
	PlanetFieldData with_biomes = make_biome_field();
	PlanetFieldData without = with_biomes;
	without.macro_map = PlanetMacroMapRef();
	without.biomes.count = 0;

	RandomPCG rng(6789);
	double worst = 0.0;
	double total = 0.0;
	const uint32_t count = 5000;
	for (uint32_t i = 0; i < count; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		if (d.length_squared() < 1e-12) {
			continue;
		}
		d.normalize();
		const double diff = Math::abs(with_biomes.eval_radius(d) - without.eval_radius(d));
		worst = MAX(worst, diff);
		total += diff;
	}
	check(worst > 100.0, "enabling biomes measurably changes the surface");
	print_line(format("        mean difference {} m, worst {} m", total / double(count), worst));
}

void check_coastal_field() {
	PlanetFieldData data = make_biome_field();
	data.sea_level_radius = data.base_radius + 600.0;
	data.coast.enabled = true;
	data.coast.noise_period = data.fbm.period * 8.f;
	data.coast.character_period = data.fbm.period * 32.f;
	const double lo = data.get_min_radius(), hi = data.get_max_radius();
	RandomPCG rng(7153);
	int violations = 0, beach_count = 0, cliff_count = 0, lod_flips = 0;
	Vector3 dirs[64];
	double sx[64], sy[64], sz[64], radii[64];
	float noise[64];
	for (int i = 0; i < 4096; ++i) {
		Vector3 d(double(rng.randf()) - 0.5, double(rng.randf()) - 0.5, double(rng.randf()) - 0.5);
		d.normalize();
		const Vector3 control = data.sample_coast(d);
		beach_count += control.y < 0.2;
		cliff_count += control.y > 0.8;
		const double r = data.eval_radius(d);
		violations += r < lo || r > hi || !Math::is_finite(r);
		violations += (r > data.sea_level_radius) != (control.x > 0.0);
		PlanetFieldData coarse = data;
		coarse.fbm.octaves = 2;
		lod_flips += (coarse.eval_radius(d) > data.sea_level_radius) != (r > data.sea_level_radius);
		if (i < 64) { dirs[i] = d; }
	}
	data.eval_radius_batch(dirs, sx, sy, sz, noise, radii, 64);
	int batch_diff = 0;
	for (int i = 0; i < 64; ++i) { batch_diff += radii[i] != data.eval_radius(dirs[i]); }
	check(violations == 0, "coast field respects bounds and the independent shoreline sign");
	check(lod_flips == 0, "changing biome octave count cannot create coastal pools or islands");
	check(batch_diff == 0, "coastal scalar and batched radii agree bit for bit");
	check(beach_count > 0 && cliff_count > 0, "independent coastal geology produces beaches and cliffs");
	PlanetFieldData changed = data;
	changed.coast.seed += 1u;
	check(!planet_field_equal(data, changed), "coast edits invalidate field/collider equality");
}

} // namespace

void run_planet_field_check() {
	print_line("============ Planet field checks (Phase 2) ============");
	g_failures = 0;

	check_scalar_vs_batched_noise();
	check_field_scalar_vs_batched();
	check_graph_matches_noise();
	check_global_bounds();
	check_cone_bounds();
	check_lipschitz_constant();

	print_line("");
	print_line("---- Phase A: shaping, derivatives, floor ----");
	check_shaped_reduces_to_plain();
	check_deriv_value_matches();
	check_analytic_derivative();
	check_shape_bounds_exact();
	check_slope_erosion_bounded();
	check_floor_clamp();
	check_smooth_terracing();
	check_shaped_scalar_vs_batched();
	check_lipschitz_with_features();

	print_line("");
	print_line("---- Phase B: macro maps and biomes ----");
	check_cubemap_roundtrip();
	check_macro_map_seam_continuity();
	check_macro_map_channels_in_range();
	check_global_biome_frequency();
	check_biome_blend_continuity();
	check_biome_weights_normalized();
	check_biome_global_bounds();
	check_biome_scalar_vs_batched();
	check_biome_lipschitz();
	check_macro_map_image_roundtrip();
	check_biomes_change_the_surface();
	check_coastal_field();

	print_line("");
	if (g_failures == 0) {
		print_line("ALL CHECKS PASSED");
	} else {
		print_line(format("{} CHECK(S) FAILED", g_failures));
	}
	print_line("======================================================");
}

} // namespace zylann::voxel::tests
