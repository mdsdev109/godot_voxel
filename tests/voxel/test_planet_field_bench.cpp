#include "test_planet_field_bench.h"

#include "../../generators/graph/voxel_generator_graph.h"
#include "../../generators/graph/voxel_graph_function.h"
#include "../../storage/voxel_buffer.h"
#include "../../util/containers/span.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/classes/fast_noise_lite.h"
#include "../../util/godot/classes/time.h"
#include "../../util/godot/core/string.h"
#include "../../util/io/log.h"
#include "../../util/math/vector3f.h"
#include "../../util/noise/fast_noise_lite/fast_noise_lite.h"
#include "../../util/string/format.h"

#ifdef VOXEL_ENABLE_FAST_NOISE_2
#include "../../util/noise/fast_noise_2.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace zylann::voxel::tests {

using namespace pg;

namespace {

// From the planetary collision design doc.
constexpr double PLANET_RADIUS_M = 3185500.0;
constexpr float VOXEL_EDGE_M = 0.25f;
// 2443 m is the ground sample distance of a 2048^2/face cubemap at this radius, i.e. the coarsest
// octave the noise stack has to supply. 13 octaves from there reaches the 0.25 m band limit.
constexpr float COARSEST_OCTAVE_PERIOD_M = 2443.f;
constexpr int NOISE_OCTAVES = 13;
constexpr float NOISE_AMPLITUDE_M = 800.f;

enum FieldVariant {
	// Sphere only. Carries no noise, so its cost is almost entirely the graph runtime's per-call
	// setup. This is the number that decides whether the narrow phase can call the graph at all.
	FIELD_SPHERE_ONLY = 0,
	// Godot's FastNoiseLite through NODE_NOISE_3D. Scalar, and a virtual call per sample.
	FIELD_GODOT_FNL,
	// ZN_FastNoiseLite through NODE_FAST_NOISE_3D. Scalar, but devirtualized.
	FIELD_ZN_FNL,
	// FastNoise2 through NODE_FAST_NOISE_2_3D. SIMD, batched via get_noise_3d_series().
	// This is the only backend that can plausibly meet the doc's 10 ns/octave/sample budget.
	FIELD_FAST_NOISE_2,
	// Custom integer-hash gradient noise, written for this project. Bit-identical in C++ and GLSL,
	// and the only backend with a chance at the design doc's 10 ns/octave budget.
	FIELD_PLANET_HASH,
	FIELD_VARIANT_COUNT
};

const char *get_variant_name(const int variant) {
	switch (variant) {
		case FIELD_SPHERE_ONLY:
			return "sphere only (no noise, isolates per-call overhead)";
		case FIELD_GODOT_FNL:
			return "Noise3D       (Godot FastNoiseLite, scalar)";
		case FIELD_ZN_FNL:
			return "FastNoise3D   (ZN_FastNoiseLite, scalar)";
		case FIELD_FAST_NOISE_2:
			return "FastNoise2_3D (FastNoise2, SIMD)";
		case FIELD_PLANET_HASH:
			return "PlanetNoise3D (custom integer hash)";
		default:
			return "?";
	}
}

// Builds `sdf(p) = (length(p) - PLANET_RADIUS) + noise(p) * amplitude`, a displaced sphere. That is
// the cheapest graph with the same shape as a real planet field.
Ref<VoxelGeneratorGraph> create_field_graph(const int variant, const int octaves) {
	Ref<VoxelGeneratorGraph> generator;
	generator.instantiate();

	Ref<VoxelGraphFunction> func = generator->get_main_function();
	if (func.is_null()) {
		print_line("Main function is null");
		return Ref<VoxelGeneratorGraph>();
	}
	VoxelGraphFunction &g = **func;

	const uint32_t n_x = g.create_node(VoxelGraphFunction::NODE_INPUT_X, Vector2());
	const uint32_t n_y = g.create_node(VoxelGraphFunction::NODE_INPUT_Y, Vector2());
	const uint32_t n_z = g.create_node(VoxelGraphFunction::NODE_INPUT_Z, Vector2());
	const uint32_t n_out = g.create_node(VoxelGraphFunction::NODE_OUTPUT_SDF, Vector2());
	const uint32_t n_sphere = g.create_node(VoxelGraphFunction::NODE_SDF_SPHERE, Vector2());

	g.add_connection(n_x, 0, n_sphere, 0);
	g.add_connection(n_y, 0, n_sphere, 1);
	g.add_connection(n_z, 0, n_sphere, 2);
	g.set_node_default_input(n_sphere, 3, float(PLANET_RADIUS_M));

	if (variant == FIELD_SPHERE_ONLY) {
		g.add_connection(n_sphere, 0, n_out, 0);

	} else {
		uint32_t n_noise = 0;

		switch (variant) {
			case FIELD_GODOT_FNL: {
				n_noise = g.create_node(VoxelGraphFunction::NODE_NOISE_3D, Vector2());
				Ref<::FastNoiseLite> noise;
				noise.instantiate();
				noise->set_noise_type(::FastNoiseLite::TYPE_SIMPLEX);
				noise->set_frequency(1.f / COARSEST_OCTAVE_PERIOD_M);
				noise->set_fractal_type(::FastNoiseLite::FRACTAL_FBM);
				noise->set_fractal_octaves(octaves);
				g.set_node_param(n_noise, 0, noise);
			} break;

			case FIELD_ZN_FNL: {
				n_noise = g.create_node(VoxelGraphFunction::NODE_FAST_NOISE_3D, Vector2());
				Ref<ZN_FastNoiseLite> noise;
				noise.instantiate();
				// ZN_FastNoiseLite has no TYPE_SIMPLEX; OpenSimplex2 is the same generator Godot's
				// FastNoiseLite calls TYPE_SIMPLEX, so the three backends stay comparable.
				noise->set_noise_type(ZN_FastNoiseLite::TYPE_OPEN_SIMPLEX_2);
				noise->set_period(COARSEST_OCTAVE_PERIOD_M);
				noise->set_fractal_type(ZN_FastNoiseLite::FRACTAL_FBM);
				noise->set_fractal_octaves(octaves);
				g.set_node_param(n_noise, 0, noise);
			} break;

#ifdef VOXEL_ENABLE_FAST_NOISE_2
			case FIELD_FAST_NOISE_2: {
				n_noise = g.create_node(VoxelGraphFunction::NODE_FAST_NOISE_2_3D, Vector2());
				Ref<FastNoise2> noise;
				noise.instantiate();
				noise->set_noise_type(FastNoise2::TYPE_OPEN_SIMPLEX_2);
				noise->set_period(COARSEST_OCTAVE_PERIOD_M);
				noise->set_fractal_type(FastNoise2::FRACTAL_FBM);
				noise->set_fractal_octaves(octaves);
				noise->update_generator();
				if (!noise->is_valid()) {
					print_line("FastNoise2 generator failed to build");
					return Ref<VoxelGeneratorGraph>();
				}
				print_line(format(
						"FastNoise2 SIMD level: {}",
						zylann::godot::to_std_string(FastNoise2::get_simd_level_name(noise->get_simd_level()))
				));
				g.set_node_param(n_noise, 0, noise);
			} break;
#endif

			case FIELD_PLANET_HASH: {
				n_noise = g.create_node(VoxelGraphFunction::NODE_PLANET_NOISE_3D, Vector2());
				g.set_node_param(n_noise, 0, 0); // seed
				g.set_node_param(n_noise, 1, COARSEST_OCTAVE_PERIOD_M); // period
				g.set_node_param(n_noise, 2, octaves);
				g.set_node_param(n_noise, 3, 2.0f); // lacunarity
				g.set_node_param(n_noise, 4, 0.5f); // gain
			} break;

			default:
				return Ref<VoxelGeneratorGraph>();
		}

		g.add_connection(n_x, 0, n_noise, 0);
		g.add_connection(n_y, 0, n_noise, 1);
		g.add_connection(n_z, 0, n_noise, 2);

		const uint32_t n_mul = g.create_node(VoxelGraphFunction::NODE_MULTIPLY, Vector2());
		g.add_connection(n_noise, 0, n_mul, 0);
		g.set_node_default_input(n_mul, 1, NOISE_AMPLITUDE_M);

		const uint32_t n_add = g.create_node(VoxelGraphFunction::NODE_ADD, Vector2());
		g.add_connection(n_sphere, 0, n_add, 0);
		g.add_connection(n_mul, 0, n_add, 1);
		g.add_connection(n_add, 0, n_out, 0);
	}

	const pg::CompilationResult result = generator->compile(false);
	if (!result.success) {
		print_line(format(
				"Failed to compile graph: node {}: {}",
				result.node_id,
				zylann::godot::to_std_string(result.message)
		));
		return Ref<VoxelGeneratorGraph>();
	}

	return generator;
}

// An orthonormal frame at an arbitrary point on the planet, so samples land on a realistic tangent
// patch instead of near the origin.
struct SurfaceFrame {
	double dir[3];
	double u[3];
	double v[3];

	SurfaceFrame() {
		const double d[3] = { 1.0, 2.0, 3.0 };
		const double dl = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
		for (int i = 0; i < 3; ++i) {
			dir[i] = d[i] / dl;
		}
		// u = normalize(cross(dir, world_up))
		const double up[3] = { 0.0, 0.0, 1.0 };
		double c[3] = { dir[1] * up[2] - dir[2] * up[1],
						dir[2] * up[0] - dir[0] * up[2],
						dir[0] * up[1] - dir[1] * up[0] };
		const double cl = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
		for (int i = 0; i < 3; ++i) {
			u[i] = c[i] / cl;
		}
		// v = cross(dir, u)
		v[0] = dir[1] * u[2] - dir[2] * u[1];
		v[1] = dir[2] * u[0] - dir[0] * u[2];
		v[2] = dir[0] * u[1] - dir[1] * u[0];
	}
};

// Fills a square patch of `count` samples spaced VOXEL_EDGE_M apart on the surface, the way a
// narrow-phase tessellation over a body's AABB footprint would.
void make_sample_patch(
		const unsigned int count,
		StdVector<float> &out_x,
		StdVector<float> &out_y,
		StdVector<float> &out_z
) {
	const SurfaceFrame frame;
	out_x.resize(count);
	out_y.resize(count);
	out_z.resize(count);

	const int side = int(std::ceil(std::sqrt(double(count))));
	const double half = 0.5 * double(side - 1) * double(VOXEL_EDGE_M);

	for (unsigned int i = 0; i < count; ++i) {
		const double su = double(int(i) % side) * double(VOXEL_EDGE_M) - half;
		const double sv = double(int(i) / side) * double(VOXEL_EDGE_M) - half;
		// Computed in double, stored as float. That is exactly what the graph runtime is handed today.
		const double px = PLANET_RADIUS_M * frame.dir[0] + su * frame.u[0] + sv * frame.v[0];
		const double py = PLANET_RADIUS_M * frame.dir[1] + su * frame.u[1] + sv * frame.v[1];
		const double pz = PLANET_RADIUS_M * frame.dir[2] + su * frame.u[2] + sv * frame.v[2];
		out_x[i] = float(px);
		out_y[i] = float(py);
		out_z[i] = float(pz);
	}
}

// Shows how much of the requested sample spacing survives float32 at planet scale. Independent of the
// timing measurement, but it constrains the same design decision.
void report_float32_resolution() {
	const unsigned int count = 64;
	StdVector<float> x, y, z;
	make_sample_patch(count, x, y, z);

	unsigned int distinct = 1;
	double min_step = 1e30;
	double max_step = 0.0;
	for (unsigned int i = 1; i < count; ++i) {
		const double dx = double(x[i]) - double(x[i - 1]);
		const double dy = double(y[i]) - double(y[i - 1]);
		const double dz = double(z[i]) - double(z[i - 1]);
		const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (step > 0.0) {
			++distinct;
			min_step = std::min(min_step, step);
		}
		max_step = std::max(max_step, step);
	}

	print_line("--- float32 resolution at planet scale ---");
	print_line(format("Planet radius            : {} m", PLANET_RADIUS_M));
	print_line(format("Requested sample spacing : {} m", VOXEL_EDGE_M));
	print_line(format("Distinct float32 positions out of {} requested : {}", count, distinct));
	print_line(format("Realized step, min / max : {} m / {} m", min_step, max_step));
	print_line("");
}

struct BenchResult {
	double ns_per_sample = 0.0;
	double us_per_call = 0.0;
};

BenchResult bench_batch_size(VoxelGeneratorGraph &generator, const unsigned int batch_size) {
	StdVector<float> x, y, z;
	make_sample_patch(batch_size, x, y, z);

	StdVector<float> out;
	out.resize(batch_size);

	const Vector3f min_pos(-1e9f, -1e9f, -1e9f);
	const Vector3f max_pos(1e9f, 1e9f, 1e9f);

	// Amortize over at least this many samples, but never fewer than this many calls. At small batch
	// sizes the per-call cost is the thing being measured, so the call count has to stay high.
	const uint64_t target_samples = 500000;
	const unsigned int min_calls = 500;
	unsigned int calls = unsigned(target_samples / batch_size);
	if (calls < min_calls) {
		calls = min_calls;
	}

	double checksum = 0.0;

	const auto run_once = [&]() -> uint64_t {
		const uint64_t t0 = Time::get_singleton()->get_ticks_usec();
		for (unsigned int i = 0; i < calls; ++i) {
			generator.generate_series(
					to_span_const(x),
					to_span_const(y),
					to_span_const(z),
					VoxelBuffer::CHANNEL_SDF,
					to_span(out),
					min_pos,
					max_pos
			);
			checksum += double(out[0]);
		}
		return Time::get_singleton()->get_ticks_usec() - t0;
	};

	// Warm up. The first calls allocate the thread-local runtime state buffers.
	run_once();

	uint64_t best_us = UINT64_MAX;
	for (int rep = 0; rep < 3; ++rep) {
		best_us = std::min(best_us, run_once());
	}

	// Keep the optimizer from removing the work.
	if (checksum == 12345.6789) {
		print_line("(unreachable)");
	}

	BenchResult r;
	r.us_per_call = double(best_us) / double(calls);
	r.ns_per_sample = (double(best_us) * 1000.0) / (double(calls) * double(batch_size));
	return r;
}

} // namespace

void run_planet_field_bench() {
	print_line("================ Planet field benchmark ================");
	print_line("");

	report_float32_resolution();

	static const unsigned int batch_sizes[] = { 8, 16, 32, 64, 128, 256, 1024, 4096 };
	const unsigned int batch_size_count = sizeof(batch_sizes) / sizeof(batch_sizes[0]);

	for (int variant = 0; variant < FIELD_VARIANT_COUNT; ++variant) {
#ifndef VOXEL_ENABLE_FAST_NOISE_2
		if (variant == FIELD_FAST_NOISE_2) {
			print_line("FastNoise2 not enabled in this build (voxel_fast_noise_2=no), skipping");
			continue;
		}
#endif
		Ref<VoxelGeneratorGraph> generator = create_field_graph(variant, NOISE_OCTAVES);
		if (generator.is_null()) {
			print_line(format("Skipping variant `{}`: graph could not be built", get_variant_name(variant)));
			continue;
		}

		print_line(format("--- {} ---", get_variant_name(variant)));
		print_line("  batch      us/call      ns/sample      vs. batch=4096");

		StdVector<BenchResult> results;
		results.resize(batch_size_count);

		double reference_ns_per_sample = 0.0;
		for (unsigned int i = 0; i < batch_size_count; ++i) {
			results[i] = bench_batch_size(**generator, batch_sizes[i]);
			if (batch_sizes[i] == 4096) {
				reference_ns_per_sample = results[i].ns_per_sample;
			}
		}

		for (unsigned int i = 0; i < batch_size_count; ++i) {
			const double ratio =
					reference_ns_per_sample > 0.0 ? results[i].ns_per_sample / reference_ns_per_sample : 0.0;
			print_line(format(
					"  {}      {}      {}      {}x",
					batch_sizes[i],
					results[i].us_per_call,
					results[i].ns_per_sample,
					ratio
			));
		}
		print_line("");
	}

	// Octave scaling. The first run showed FastNoise2 (SIMD) coming out slower than scalar
	// FastNoiseLite, which contradicts the design doc's assumption. If cost scales linearly with
	// octave count the measurement is real; if it is flat, the fractal settings are not reaching the
	// generator and the comparison is invalid.
	{
		static const int octave_counts[] = { 1, 2, 4, 8, 13 };
		const unsigned int octave_count_count = sizeof(octave_counts) / sizeof(octave_counts[0]);
		const unsigned int octave_bench_batch = 64;

		print_line(format("--- octave scaling at batch={} ---", octave_bench_batch));
		print_line("  backend / octaves      ns/sample      ns/octave/sample");

		for (int variant = FIELD_GODOT_FNL; variant < FIELD_VARIANT_COUNT; ++variant) {
#ifndef VOXEL_ENABLE_FAST_NOISE_2
			if (variant == FIELD_FAST_NOISE_2) {
				continue;
			}
#endif
			for (unsigned int i = 0; i < octave_count_count; ++i) {
				Ref<VoxelGeneratorGraph> generator = create_field_graph(variant, octave_counts[i]);
				if (generator.is_null()) {
					continue;
				}
				const BenchResult r = bench_batch_size(**generator, octave_bench_batch);
				print_line(format(
						"  {} / {}      {}      {}",
						get_variant_name(variant),
						octave_counts[i],
						r.ns_per_sample,
						r.ns_per_sample / double(octave_counts[i])
				));
			}
		}
		print_line("");
	}

	print_line("Decision rule: compare ns/sample at batch=32 against batch=4096.");
	print_line("  Within ~2x  -> the narrow phase can call the graph directly.");
	print_line("  ~10x worse  -> per-call setup dominates. Hand-write the hot path and keep the graph");
	print_line("                 for analyze_range() and get_shader_source() only.");
	print_line("=======================================================");
}

} // namespace zylann::voxel::tests
