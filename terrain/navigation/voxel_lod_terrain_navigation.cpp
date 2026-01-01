#include "voxel_lod_terrain_navigation.h"
#include "../../util/godot/classes/engine.h"
#include "../../util/godot/classes/navigation_mesh_source_geometry_data_3d.h"
#include "../../util/godot/classes/navigation_region_3d.h"
#include "../../util/godot/classes/navigation_server_3d.h"
#include "../../util/godot/classes/object.h"
#include "../../util/godot/core/packed_arrays.h"
#include "../../util/math/conv.h"
#include "../../util/profiling.h"
#include "../variable_lod/voxel_lod_terrain.h"
#include "core/math/geometry_3d.h"

#ifdef DEBUG_ENABLED
#include "../../util/string/format.h"
#endif

namespace zylann::voxel {

namespace {

#ifdef DEV_ENABLED

void dump_navmesh_source_data_to_mesh_resource(
		const PackedFloat32Array &vertices,
		const PackedInt32Array &indices,
		const char *fpath
) {
	PackedVector3Array debug_vertices;
	for (int i = 0; i < vertices.size(); i += 3) {
		debug_vertices.push_back(Vector3(vertices[i], vertices[i + 1], vertices[i + 2]));
	}

	PackedInt32Array debug_indices = indices;
	for (int i0 = 1; i0 < debug_indices.size(); i0 += 3) {
		std::swap(debug_indices.write[i0], debug_indices.write[i0 + 1]);
	}

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = debug_vertices;
	arrays[Mesh::ARRAY_INDEX] = debug_indices;
	Ref<ArrayMesh> debug_mesh;
	debug_mesh.instantiate();
	debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	ResourceSaver::save(debug_mesh, fpath);
}

void check_navmesh_source_data(
		const PackedFloat32Array &vertices,
		const PackedInt32Array &indices,
		unsigned int vertex_count
) {
	ZN_PROFILE_SCOPE();
	// Some sanity checks to make sure we don't send garbage to the baker

	for (int ii = 0; ii < indices.size(); ++ii) {
		const int32_t i = indices[ii];
		ZN_ASSERT(i >= 0 && i < vertices.size());
	}

	unsigned int used_vertices_count = 0;
	StdVector<bool> used_vertices;
	used_vertices.resize(vertices.size(), false);
	for (int ii = 0; ii < indices.size(); ++ii) {
		const int32_t i = indices[ii];
		if (!used_vertices[i]) {
			used_vertices[i] = true;
			++used_vertices_count;
		}
	}
	// Can't actually expect all vertices to be used, our implementation of Transvoxel has edge cases that can leave a
	// few vertices orphaned (nearly-degenerate triangles). So we'll check with reasonable margin instead
	ZN_ASSERT(used_vertices_count > (10 * vertex_count / 11));
}

size_t get_geometry_cache_estimated_size_in_bytes(const VoxelMeshMap<VoxelMeshBlockVLT> &map) {
	size_t size = 0;
	map.for_each_block([&size](const VoxelMeshBlockVLT &block) { //
		size += get_estimated_size_in_bytes(block.geometry_cache);
	});
	return size;
}

#endif

void copy_navigation_mesh_settings(NavigationMesh &dst, const NavigationMesh &src) {
	dst.set_agent_height(src.get_agent_height());
	dst.set_spherical(src.get_spherical());
	dst.set_agent_max_climb(src.get_agent_max_climb());
	dst.set_agent_max_slope(src.get_agent_max_slope());
	// TODO `get_agent_radius` is not `const` due to an oversight in Godot
	dst.set_agent_radius(const_cast<NavigationMesh &>(src).get_agent_radius());

	// These are tricky because they must match the map... Instead of requiring the user to set them, we'll
	// automatically use the map's values instead.
	//
	dst.set_cell_height(src.get_cell_height());
	dst.set_cell_size(src.get_cell_size());

	dst.set_detail_sample_distance(src.get_detail_sample_distance());
	dst.set_detail_sample_max_error(src.get_detail_sample_max_error());
	dst.set_edge_max_error(src.get_edge_max_error());
	dst.set_edge_max_length(src.get_edge_max_length());
	dst.set_filter_baking_aabb(src.get_filter_baking_aabb());
	dst.set_filter_baking_aabb_offset(src.get_filter_baking_aabb_offset());
	dst.set_filter_ledge_spans(src.get_filter_ledge_spans());
	dst.set_collision_mask(src.get_collision_mask());
	dst.set_parsed_geometry_type(src.get_parsed_geometry_type());
	dst.set_source_geometry_mode(src.get_source_geometry_mode());
	dst.set_source_group_name(src.get_source_group_name());
	dst.set_region_merge_size(src.get_region_merge_size());
	dst.set_region_min_size(src.get_region_min_size());
	dst.set_sample_partition_type(src.get_sample_partition_type());
	dst.set_vertices_per_polygon(src.get_vertices_per_polygon());
}

void flip_triangle_indices_winding(Span<int32_t> indices) {
	ZN_PROFILE_SCOPE();
	for (unsigned int i0 = 1; i0 < indices.size(); i0 += 3) {
		std::swap(indices[i0], indices[i0 + 1]);
	}
}

Ref<NavigationMeshSourceGeometryData3D> gather_geometry(const VoxelLodTerrain &terrain) {
	ZN_PROFILE_SCOPE();

	struct VoxelMeshGeometry {
		PackedVector3Array vertices;
		PackedInt32Array indices;
		Vector3i block_position;
	};

	StdVector<VoxelMeshGeometry> geoms;

	{
		ZN_PROFILE_SCOPE_NAMED("Gather from terrain");

		const VoxelMeshMap<VoxelMeshBlockVLT> &mesh_map = terrain.get_mesh_map(0);
		// TODO Could benefit from TempAllocator
		geoms.reserve(100);

		// Gather geometry from voxel terrain
		mesh_map.for_each_block([&geoms](const VoxelMeshBlockVLT &block) {
			for (const VoxelMeshGeometryCache::Surface &surface : block.geometry_cache.surfaces) {
				geoms.push_back(VoxelMeshGeometry{ surface.vertices, surface.indices, block.position });
			}
		});

#ifdef DEBUG_ENABLED
		{
			ZN_PROFILE_SCOPE_NAMED("Size estimation");
			const size_t cache_size = get_geometry_cache_estimated_size_in_bytes(mesh_map);
			ZN_PRINT_VERBOSE(format("Mesh map geometry cache size: {}", cache_size));
		}
#endif
	}

	// TODO Also gather props
	// TODO Have the option to combine data with Godot's scene tree parser?
	// `NavigationServer::parse_source_geometry_data`

	Ref<NavigationMeshSourceGeometryData3D> navmesh_source_data;
	{
		ZN_PROFILE_SCOPE_NAMED("Processing");

		unsigned int vertex_count = 0;
		for (const VoxelMeshGeometry &geom : geoms) {
			vertex_count += geom.vertices.size();
		}

		unsigned int index_count = 0;
		for (const VoxelMeshGeometry &geom : geoms) {
			index_count += geom.indices.size();
		}

		PackedFloat32Array vertices;
		PackedInt32Array indices;

		// Resize in one go, it's faster than many push_backs
		vertices.resize(vertex_count * 3);
		indices.resize(index_count);

		Span<Vector3f> vertices_w = to_span(vertices).reinterpret_cast_to<Vector3f>();
		Span<int32_t> indices_w = to_span(indices);

		const int block_size = terrain.get_mesh_block_size();
		int32_t vertex_start = 0;

		// Combine meshes
		for (const VoxelMeshGeometry &geom : geoms) {
			Span<const Vector3> vertices_r = to_span(geom.vertices);
			Span<const int32_t> indices_r = to_span(geom.indices);

			// Chunk meshes are all aligned in a grid, so transforming vertices is simple
			const Vector3f origin = to_vec3f(geom.block_position * block_size);

			for (unsigned int vi = 0; vi < vertices_r.size(); ++vi) {
				const Vector3f v = to_vec3f(vertices_r[vi]) + origin;
				vertices_w[vi] = v;
			}

			for (unsigned int ii = 0; ii < indices_r.size(); ++ii) {
				indices_w[ii] = indices_r[ii] + vertex_start;
			}

			vertex_start += vertices_r.size();
			// Move spans forward by the amount of vertices and indices we just added
			vertices_w = vertices_w.sub(vertices_r.size());
			indices_w = indices_w.sub(indices_r.size());
		}

		// Reset span to cover all indices
		indices_w = to_span(indices);

		// For some reason winding is flipped, perhaps that's a requirement from Recast
		flip_triangle_indices_winding(indices_w);

#ifdef DEV_ENABLED
		if (geoms.size() > 0) {
			check_navmesh_source_data(vertices, indices, vertex_count);
		}
#endif

		navmesh_source_data.instantiate();
		// Directly set vertices, don't use any of the other methods, they do more work and aren't specialized to our
		// case
		navmesh_source_data->append_arrays(vertices, indices);
	}

	return navmesh_source_data;
}

StdVector<const VoxelMeshBlockVLT *> get_neighboring_blocks(
		const VoxelLodTerrain &terrain,
		const Vector3i &source_block_pos
) {
	StdVector<const VoxelMeshBlockVLT *> neighboring_blocks;
	neighboring_blocks.reserve(8);

	const VoxelMeshMap<VoxelMeshBlockVLT> &mesh_map = terrain.get_mesh_map(0);

	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(0, 0, 1)));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(0, 1, 0)));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(0, 1, 1)));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(1, 0, 0)));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(1, 0, 1)));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(1, 1, 0)));
	neighboring_blocks.push_back(mesh_map.get_block(source_block_pos + Vector3i(1, 1, 1)));

	return neighboring_blocks;
}

Ref<NavigationMeshSourceGeometryData3D> gather_geometry(
		NavigationMesh::ParsedGeometryType parsed_geometry_type,
		const StdVector<const VoxelMeshBlockVLT *> &neighboring_blocks,
		const Transform3D &source_transform
) {
	ZN_PROFILE_SCOPE();

	ZN_PROFILE_SCOPE_NAMED("Gather from terrain");

	Ref<NavigationMeshSourceGeometryData3D> source_geometry_data;

	source_geometry_data.instantiate();
	source_geometry_data->clear();
	source_geometry_data->root_node_transform = Transform3D(); // global_block_transform;

	// Gather geometry from voxel terrain
	for (const VoxelMeshBlockVLT *block : neighboring_blocks) {
		if (block != nullptr) {
			Ref<Mesh> mesh = block->get_mesh();
			if (mesh.is_valid()) {
				// TODO Could optimize by reading directly from the Mesh instead of going through the geometry cache
				// but this is simpler and not a big deal because this function is not called often
				// (only when a navmesh must be generated or regenerated)
				if (parsed_geometry_type == NavigationMesh::PARSED_GEOMETRY_MESH_INSTANCES ||
					parsed_geometry_type == NavigationMesh::PARSED_GEOMETRY_BOTH) {
					source_geometry_data->add_mesh(
							mesh, source_transform.affine_inverse() * block->get_transform()
					); // global_block_transform.affine_inverse() * block->get_transform());
				}
			}
		}
	}

	return source_geometry_data;
}

} // namespace

VoxelLodTerrainNavigation::VoxelLodTerrainNavigation() {
	ZN_PRINT_VERBOSE("Construct VoxelLodTerrainNavigation");
	update_processing_state();
}

VoxelLodTerrainNavigation::~VoxelLodTerrainNavigation() {
	ZN_PRINT_VERBOSE("Destroy VoxelLodTerrainNavigation");
	// Nodes will be automatically freed when removed from scene tree
	// But you might want to clear your vectors
	MutexLock reglock(nav_region_mutex);
	MutexLock queuelock(nav_queue_mutex);
	process_queue.clear();
	_available_nav_regions.clear();
	_nav_region_active_pool.clear();
}

void VoxelLodTerrainNavigation::set_enabled(bool p_enabled) {
	_enabled = p_enabled;
	update_processing_state();
}

void VoxelLodTerrainNavigation::set_run_in_editor(bool enable) {
	_run_in_editor = enable;
	update_processing_state();
}

bool VoxelLodTerrainNavigation::get_run_in_editor() const {
	return _run_in_editor;
}

void VoxelLodTerrainNavigation::update_processing_state() {
	bool enabled = _enabled;
#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint() && !_run_in_editor) {
		enabled = false;
	}
#endif
	set_process_internal(enabled);
}

bool VoxelLodTerrainNavigation::is_enabled() const {
	return _enabled;
}

void VoxelLodTerrainNavigation::set_template_navigation_mesh(Ref<NavigationMesh> navmesh) {
	_template_navmesh = navmesh;
#ifdef TOOLS_ENABLED
	update_configuration_warnings();
#endif
}

Ref<NavigationMesh> VoxelLodTerrainNavigation::get_template_navigation_mesh() const {
	return _template_navmesh;
}

void VoxelLodTerrainNavigation::debug_set_regions_visible_in_editor(bool enable) {
	if (_visible_regions_in_editor == enable) {
		return;
	}
	_visible_regions_in_editor = enable;
#ifdef TOOLS_ENABLED
	// Don't forget to turn this off before saving, or you'll end up saving runtime-generated nodes.
	if (is_inside_tree()) {
		if (_visible_regions_in_editor) {
		} else {
		}

		// TODO The Godot Editor doesn't update the scene tree dock when the owner of a node changes.
		// One workaround is to switch to another scene tab and back.
		// Another workaround is to instantiate a node owned by the edited scene, and delete it immediately after,
		// which does update the scene tree, but DOES NOT update navmesh previews. And when I set the owner back to
		// null, navmesh previews don't go away until I tab in and out of the scene...
		//
		// Node *temp_node = memnew(Node);
		// add_child(temp_node);
		// temp_node->set_owner(get_tree()->get_edited_scene_root());
		// temp_node->queue_free();
	}
#endif
}

bool VoxelLodTerrainNavigation::debug_get_regions_visible_in_editor() const {
	return _visible_regions_in_editor;
}

void VoxelLodTerrainNavigation::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_INTERNAL_PROCESS:
			process(get_process_delta_time());
			break;
		default:
			break;
	}
}

void VoxelLodTerrainNavigation::process(float delta_time) {
	ZN_PROFILE_SCOPE();

	if (!_enabled) {
		return;
	}

#ifdef TOOLS_ENABLED
	if (Engine::get_singleton()->is_editor_hint() && !_run_in_editor) {
		return;
	}
#endif

	if (_template_navmesh.is_null()) {
		return;
	}

	Node *parent_node = get_parent();
	ZN_ASSERT_RETURN(parent_node != nullptr);

	VoxelLodTerrain *terrain = Object::cast_to<VoxelLodTerrain>(parent_node);
	if (terrain == nullptr) {
		return;
	}

	process_nav_meshes(*terrain, delta_time);
}

Vector3 GetBlockCenter(
		Transform3D volume_transform,
		Vector3i block_pos,
		int mesh_block_size,
		bool cube_sphere,
		int radius
) {
	if (!cube_sphere) {
		Vector3 blockCenter = volume_transform.xform(
				to_vec3(block_pos * mesh_block_size + Vector3iUtil::create(mesh_block_size / 2))
		);
		return blockCenter;
	} else {
		/*
		Vector3 blockCenterNoTrans =
				to_vec3(block->position * mesh_block_size + Vector3iUtil::create(mesh_block_size / 2));

		blockCenterNoTrans.x += (block->offset.x - radius);
		blockCenterNoTrans.y += (block->offset.y - radius);
		blockCenterNoTrans.z += (block->offset.z - radius);
		Vector3 blockCenter = volume_transform.xform(blockCenterNoTrans);
		return blockCenter;
		*/
		return Vector3();
	}
}

Ref<NavigationMesh> VoxelLodTerrainNavigation::CreateNavMesh() {
	Ref<NavigationMesh> navmesh = memnew(NavigationMesh);

	// Not using `duplicate` because it is much slower. We could also recycle navmeshes?
	if (_template_navmesh.is_valid()) {
		copy_navigation_mesh_settings(**navmesh, **_template_navmesh);
	}
	return navmesh;
}

NavigationRegion3D *VoxelLodTerrainNavigation::get_available_nav_region() {
	MutexLock lock(nav_region_mutex);
	NavigationRegion3D *available_nav_region = nullptr;
	if (_available_nav_regions.empty()) {
		// create two new nodes the pool must grow ideally this will end up with 1 extra all the time
		for (int i = 0; i < 3; i++) {
			NavigationRegion3D *new_nav_region = memnew(NavigationRegion3D);
			new_nav_region->set_name("lodNavRegion");
			new_nav_region->set_visible(true);
			new_nav_region->set_navigation_mesh(CreateNavMesh());
			add_child(new_nav_region, true);
			new_nav_region->set_owner(get_owner());

			_available_nav_regions.push_back(new_nav_region);
		}
	}

	available_nav_region = _available_nav_regions[0];
	_available_nav_regions.erase(_available_nav_regions.begin());
	return available_nav_region;
}

void VoxelLodTerrainNavigation::GenerateNavMesh(const VoxelLodTerrain &terrain, Vector3i block_pos) {
	if (_nav_region_active_pool.find(block_pos) == _nav_region_active_pool.end()) {
		NavRegionData nrd;
		nrd.region = nullptr;
		nrd.current_marker = 0;
		_nav_region_active_pool.insert({ block_pos, nrd });
		printf("Inserting new nav region for block %s\n", String(block_pos).utf8().get_data());
	}

	StdVector<const VoxelMeshBlockVLT *> neighbors = get_neighboring_blocks(terrain, block_pos);
	bool skip = true;
	for (const VoxelMeshBlockVLT *b : neighbors) {
		if (b != nullptr) {
			if (b->has_mesh()) {
				skip = false;
				break;
			}
		}
	}

	// We don't actually need to populate any of the data if there are no real blocks
	if (skip) {
		return;
	}

	NavRegionData &nrd = _nav_region_active_pool[block_pos];
	if (nrd.region == nullptr) {
		nrd.region = get_available_nav_region();
	}

	uint8_t marker_val = 0;
	for (auto neighbor_block : neighbors) {
		if (neighbor_block != nullptr) {
			marker_val += neighbor_block->get_marker();
		}
	}

	nrd.current_marker = marker_val;

	NavigationRegion3D *_navigation_region_ref = nrd.region;
	Ref<NavigationMesh> navMesh = _navigation_region_ref->get_navigation_mesh();
	// Ref<NavigationMesh> navMesh = CreateNavMesh();
	//_navigation_region_ref->set_navigation_mesh(navMesh);
	int block_size = terrain.get_mesh_block_size();

	Transform3D block_transform = terrain.get_global_transform() *
			Transform3D(Basis(), to_vec3(block_pos * block_size)); // * block->get_transform();
	Transform3D global_block_transform = block_transform;
	_navigation_region_ref->set_global_transform(global_block_transform);
	Ref<NavigationMeshSourceGeometryData3D> source_geometry_data =
			gather_geometry(navMesh->get_parsed_geometry_type(), neighbors, global_block_transform);
	// navMesh->set_up(block->get_transform().get_basis().get_column(1).normalize());
	navMesh->clear();

	int margin = 4; // padding

	// Filter size = block plus margin both sides.
	Vector3 filter_size(block_size + 2 * margin, block_size + 2 * margin, block_size + 2 * margin);

	// Filter origin = always (0,0,0).
	AABB filter_aabb(Vector3(0, 0, 0), filter_size);

	// Offset = centers the filter on the current block inside the 2*block_size bake.
	// That’s just half the block size in each dimension.
	Vector3 filter_offset(block_size / 2 - margin, block_size / 2 - margin, block_size / 2 - margin);

	// Boundary trims back the margin.
	float boundary_size = static_cast<float>(margin);

	// ---- Apply ----
	navMesh->set_filter_baking_aabb(filter_aabb);
	navMesh->set_filter_baking_aabb_offset(filter_offset);
	navMesh->set_border_size(boundary_size);

	/*
	source_geometry_data.instantiate();
	source_geometry_data->clear();
	source_geometry_data->root_node_transform = Transform3D();// global_block_transform;
	NavigationMesh::ParsedGeometryType parsed_geometry_type = navMesh->get_parsed_geometry_type();

	if (parsed_geometry_type == NavigationMesh::PARSED_GEOMETRY_MESH_INSTANCES ||
			parsed_geometry_type == NavigationMesh::PARSED_GEOMETRY_BOTH) {
		source_geometry_data->add_mesh(bMesh, Transform3D());//global_block_transform.affine_inverse() *
	block->get_transform());
	}
*/
	if (true) {
		// ZN_PRINT_WARNING("Baking navmesh!");
		// Bind to a local handler so we can post-process the baked NavigationMesh
		// (clip geometry on the Y axis) before forwarding to the region's
		// _bake_finished method.
		NavigationServer3D::get_singleton()->bake_from_source_geometry_data_async(
				navMesh,
				source_geometry_data,
				callable_mp(this, &VoxelLodTerrainNavigation::on_region_bake_finished)
						.bind(_navigation_region_ref, navMesh)
		);

	} else {
		// Synchronous path: call the synchronous bake API but still bind our
		// local handler so post-processing happens consistently.
		NavigationServer3D::get_singleton()->bake_from_source_geometry_data(
				navMesh,
				source_geometry_data,
				callable_mp(this, &VoxelLodTerrainNavigation::on_region_bake_finished)
						.bind(_navigation_region_ref, navMesh)
		);
	}
}


Vector3 round_to_hundredth(const Vector3 &v) {
    return Vector3(
        std::round(v.x * 100.0f) / 100.0f,
        std::round(v.y * 100.0f) / 100.0f,
        std::round(v.z * 100.0f) / 100.0f
    );
}

void VoxelLodTerrainNavigation::on_region_bake_finished(NavigationRegion3D *region, Ref<NavigationMesh> navmesh) {
	ZN_PROFILE_SCOPE();

	if (region == nullptr || navmesh.is_null()) {
		return;
	}

	// TODO: add Y-axis clipping/post-processing logic here if desired. We do
	// not implement the clipping here because the exact behavior (how to
	// re-triangulate clipped polygons, tolerances, etc.) depends on the
	// caller's requirements.

	// Clip geometry in Y using navmesh's filter AABB and border_size.
	// The filter_aabb describes the full area used for baking; border_size
	// is the padding we want to remove from top and bottom so that neighboring
	// sectors line up. We'll trim 'border_size' from both top and bottom.

	Vector<Vector3> old_vertices;
	Vector<Vector<int>> old_polys;
	navmesh->get_data(old_vertices, old_polys);

	AABB filter_aabb = navmesh->get_filter_baking_aabb();
	Vector3 filter_offset = navmesh->get_filter_baking_aabb_offset();
	float border = navmesh->get_border_size();

	const float min_y = filter_aabb.position.y + filter_offset.y + border;
	const float max_y = filter_aabb.position.y + filter_aabb.size.y + filter_offset.y - border;

	printf("Navmesh clip Y: [%.2f, %.2f]\n", min_y, max_y);
	float lowest_y = 100000000.f;
	float highest_y = -100000000.f;
	for (int i = 0; i < old_vertices.size(); ++i) {
		if (old_vertices[i].y < lowest_y) {
			lowest_y = old_vertices[i].y;
		}
		if (old_vertices[i].y > highest_y) {
			highest_y = old_vertices[i].y;
		}
	}
	if (true){//lowest_y < min_y || highest_y > max_y) {
		printf(" - before clipping: navmesh Y range is [%.2f, %.2f]\n", lowest_y, highest_y);

		Vector<Vector3> new_vertices;
		Vector<Vector<int>> new_polys;

		// For simplicity we will not attempt to reuse vertices between triangles.
		// Each clipped triangle will be emitted as independent vertices and a
		// triangle polygon referencing them. This is straightforward and avoids
		// costly deduplication/reindexing.

		for (int pi = 0; pi < old_polys.size(); ++pi) {
			const Vector<int> &poly = old_polys[pi];
			// Collect polygon points
			Vector<Vector3> poly_pts;
			for (int k = 0; k < poly.size(); ++k) {
				int vid = poly[k];
				if (vid >= 0 && vid < old_vertices.size()) {
					poly_pts.push_back(round_to_hundredth(old_vertices[vid]));
				} else {
					poly_pts.push_back(Vector3());
					printf("Warning: invalid vertex index %d (max %d)\n", vid, old_vertices.size() - 1);
				}
			}

			bool needs_clipping = false;
			for (int k = 0; k < poly_pts.size(); ++k) {
				if (poly_pts[k].y < min_y || poly_pts[k].y > max_y) {
					// This polygon needs clipping
					needs_clipping = true;
					printf("polyPoint %d = (%.2f, %.2f, %.2f) [%.2f, %.2f] needs clipping\n",
						   k,
						   poly_pts[k].x,
						   poly_pts[k].y,
						   poly_pts[k].z, min_y, max_y);
					break;
				}
			}
			needs_clipping = false;
			if (!needs_clipping) {
				// Fast path: just copy the polygon as-is
				int base = new_vertices.size();
				for (int k = 0; k < poly_pts.size(); ++k) {
					new_vertices.push_back(poly_pts[k]);
				}
				Vector<int> new_poly;
				for (int k = 0; k < poly.size(); ++k) {
					new_poly.push_back(base + k);
				}
				new_polys.push_back(new_poly);
			} 
			else 
			{
				// Needs clipping

				// Clip against top plane (y <= max_y) then bottom plane (y >= min_y).
				for (int k = 0; k < poly_pts.size(); ++k) {
					printf(" - polyPoint %d = (%.2f, %.2f, %.2f)\n", k, poly_pts[k].x, poly_pts[k].y, poly_pts[k].z);
				}
				Vector<Vector3> clipped =
						Geometry3D::clip_polygon(poly_pts, Plane(Vector3(0, 1, 0), Vector3(0, max_y, 0)));
				printf(" - clipped against top plane ( %.1f ) , %d points remain\n", max_y, clipped.size());
				for (int k = 0; k < clipped.size(); ++k) {
					printf("   - clippedPoint %d = (%.2f, %.2f, %.2f)\n", k, clipped[k].x, clipped[k].y, clipped[k].z);
				}
				clipped = Geometry3D::clip_polygon(clipped, Plane(Vector3(0, -1, 0), Vector3(0, min_y, 0)));
				printf(" - clipped against bottom plane ( %.1f ) , %d points remain\n", min_y, clipped.size());
				for (int k = 0; k < clipped.size(); ++k) {
					printf("   - clippedPoint %d = (%.2f, %.2f, %.2f)\n", k, clipped[k].x, clipped[k].y, clipped[k].z);
				}
				// Triangulate clipped polygon (fan triangulation)
				if (clipped.size() < 3) {
					continue;
				}
				for (int ti = 1; ti < clipped.size() - 1; ++ti) {
					int base = new_vertices.size();
					new_vertices.push_back(round_to_hundredth(clipped[0]));
					new_vertices.push_back(round_to_hundredth(clipped[ti]));
					new_vertices.push_back(round_to_hundredth(clipped[ti + 1]));
					Vector<int> tri;
					tri.push_back(base);
					tri.push_back(base + 1);
					tri.push_back(base + 2);
					new_polys.push_back(tri);
				}
			}
		}
		lowest_y = 100000000.f;
		highest_y = -100000000.f;
		for (int i = 0; i < new_vertices.size(); ++i) {
			if (new_vertices[i].y < lowest_y) {
				lowest_y = new_vertices[i].y;
			}
			if (new_vertices[i].y > highest_y) {
				highest_y = new_vertices[i].y;
			}
		}
		printf(" - after clipping: navmesh Y range is [%.2f, %.2f]\n", lowest_y, highest_y);

		printf(" - clipped navmesh has %d vertices and %d polys (was %d vertices and %d polys)\n",
			   new_vertices.size(),
			   new_polys.size(),
			   old_vertices.size(),
			   old_polys.size());

		if (!new_polys.is_empty()) {
			// Replace navmesh geometry with clipped/triangulated data.
			navmesh->set_data(new_vertices, new_polys);
		}
	}
	// Forward to the region's original bake finished handler so navigation can
	// integrate the (possibly post-processed) navmesh.
	Callable cb = callable_mp(region, &NavigationRegion3D::_bake_finished).bind(navmesh);
	cb.call();
}

void VoxelLodTerrainNavigation::process_nav_meshes(const VoxelLodTerrain &terrain, float delta) {
	ZN_PROFILE_SCOPE();

	nav_queue_mutex.lock();

	_time_before_update += 1.0;
	static float debug_time = 0.0f;
	debug_time += delta;

	if (!process_queue.empty() && _time_before_update >= 2.0) {
		Vector3i block_pos = process_queue.front();
		process_queue.pop_front();
		GenerateNavMesh(terrain, block_pos);
		nav_queue_mutex.unlock();
		_time_before_update = 0.0;
	} else {
		nav_queue_mutex.unlock();
	}
	std::vector<std::vector<Vector3i>> viewerNavVectors;
	// TODO Support for multiple viewers, this is a placeholder implementation
	VoxelEngine::get_singleton().for_each_viewer([&viewerNavVectors,
												  &terrain](ViewerID id, const VoxelEngine::Viewer &viewer) { //
		std::vector<Vector3i> sortVector;
		for (unsigned int lod_index = 0; lod_index < 1; ++lod_index) {
			const VoxelMeshMap<VoxelMeshBlockVLT> &mesh_map = terrain.get_mesh_map(lod_index);

			mesh_map.for_each_block([&sortVector](const VoxelMeshBlockVLT &block) { //
				Vector3i block_pos = block.position;
				for (int dx = -1; dx <= 1; dx++) {
					for (int dy = -1; dy <= 1; dy++) {
						for (int dz = -1; dz <= 1; dz++) {
							Vector3i n_pos = block_pos + Vector3i(dx, dy, dz);
							if (std::find(sortVector.begin(), sortVector.end(), n_pos) != sortVector.end()) {
								continue;
							}
							sortVector.push_back(n_pos);
						}
					}
				}
			});
		}
		std::sort(begin(sortVector), end(sortVector), [&viewer, &terrain](Vector3i lhs, Vector3i rhs) {
			Transform3D volume_transform = terrain.get_global_transform();
			const int mesh_block_size = terrain.get_mesh_block_size();
			Vector3 offset = Vector3(mesh_block_size, mesh_block_size, mesh_block_size);
			Vector3 lhs_nav_center = (lhs * mesh_block_size) + offset;
			Vector3 rhs_nav_center = (rhs * mesh_block_size) + offset;
			/*
			Vector3 lhs_block_center =
					GetBlockCenter(volume_transform, lhs, mesh_block_size, false, 0); //_cube_sphere, get_radius());
			Vector3 rhs_block_center =
					GetBlockCenter(volume_transform, rhs, mesh_block_size, false, 0); //_cube_sphere, get_radius());
			*/

			return viewer.world_position.distance_to(lhs_nav_center) <
					viewer.world_position.distance_to(rhs_nav_center);
		});
		viewerNavVectors.push_back(sortVector);
	});

	StdVector<Vector3i> in_use_block_positions;
	for (int i = 0; i < viewerNavVectors.size(); i++) {
		int in_use = 0;
		// printf("Sorted list for user %d:\n", i);
		for (int j = 0; j < viewerNavVectors[i].size(); j++) {
			Vector3i block_pos = viewerNavVectors[i][j];
			// printf("   %d Block %s\n", j, String(block_pos).utf8().get_data());
			if (in_use < 48) {
				auto it = _nav_region_active_pool.find(block_pos);
				if (it == _nav_region_active_pool.end()) {
					nav_queue_mutex.lock();
					if (std::find(process_queue.begin(), process_queue.end(), block_pos) == process_queue.end()) {
						process_queue.push_back(block_pos);
						printf(" Adding navmesh block %s to the queue\n", String(block_pos).utf8().get_data());
					}
					nav_queue_mutex.unlock();
				}
			}

			if (std::find(in_use_block_positions.begin(), in_use_block_positions.end(), block_pos) ==
				in_use_block_positions.end()) {
				in_use_block_positions.push_back(block_pos);
			}
			in_use++;

			if (in_use >= 72) {
				break;
			}
		}
	}

	StdVector<Vector3i> blocks_to_clean;
	for (auto it = _nav_region_active_pool.begin(); it != _nav_region_active_pool.end(); it++) {
		Vector3i block_pos = it->first;
		bool found = false;
		for (Vector3i in_use_pos : in_use_block_positions) {
			if (block_pos == in_use_pos) {
				found = true;
				break;
			}
		}
		if (!found) {
			blocks_to_clean.push_back(block_pos); // Collect first
		}
	}

	// Now safely clean the blocks
	for (const Vector3i &block_pos : blocks_to_clean) {
		clean_nav_region(block_pos);
	}

	if (debug_time >= 5.0f) {
		nav_queue_mutex.lock();

		for (int i = 0; i < in_use_block_positions.size(); i++) {
			printf(" In use block [%d]: %s\n", i, String(in_use_block_positions[i]).utf8().get_data());
		}

		for (auto it = _nav_region_active_pool.begin(); it != _nav_region_active_pool.end(); it++) {
			Vector3i block_pos = it->first;
			printf(" Active pool block: %s\n", String(block_pos).utf8().get_data());
		}
		for (int i = 0; i < process_queue.size(); i++) {
			Vector3i block_pos = process_queue[i];
			printf(" Queue block: %s\n", String(block_pos).utf8().get_data());
		}
		nav_queue_mutex.unlock();

		debug_time = 0.0f;
	}

	// Check for any changes
	for (auto it = _nav_region_active_pool.begin(); it != _nav_region_active_pool.end(); it++) {
		Vector3i block_pos = it->first;
		NavRegionData &nrd = it->second;

		uint8_t marker_val = 0;
		StdVector<const VoxelMeshBlockVLT *> neighbors = get_neighboring_blocks(terrain, block_pos);
		for (auto neighbor_block : neighbors) {
			if (neighbor_block != nullptr) {
				marker_val += neighbor_block->get_marker();
			}
		}

		if (marker_val != nrd.current_marker) {
			printf(" Navmesh block %s marker changed from %d to %d\n",
				   String(block_pos).utf8().get_data(),
				   nrd.current_marker,
				   marker_val);
			nrd.current_marker = marker_val;
			nav_queue_mutex.lock();
			if (std::find(process_queue.begin(), process_queue.end(), block_pos) == process_queue.end()) {
				process_queue.push_back(block_pos);
				printf(" Adding navmesh block %s to the queue\n", String(block_pos).utf8().get_data());
			}
			nav_queue_mutex.unlock();
		}
	}
}

void VoxelLodTerrainNavigation::clean_nav_region(Vector3i block) {
	printf(" Cleaning navmesh block %s\n", String(block).utf8().get_data());
	if (_nav_region_active_pool.find(block) == _nav_region_active_pool.end()) {
		printf(" Couldn't find block %s to clean.\n", String(block).utf8().get_data());
		return;
	}

	// ZN_PRINT_WARNING("Cleaning block region nav mesh");
	NavRegionData &nrd = _nav_region_active_pool[block];
	if (nrd.region != nullptr) {
		nrd.region->get_navigation_mesh()->clear();
		MutexLock lock(nav_region_mutex);
		_available_nav_regions.push_back(nrd.region);
		nrd.region = nullptr;
	}
	_nav_region_active_pool.erase(block);
}

void VoxelLodTerrainNavigation::bake(const VoxelLodTerrain &terrain) {
	/*
		ZN_PROFILE_SCOPE();
		Ref<NavigationMeshSourceGeometryData3D> navmesh_source_data = gather_geometry(terrain);

		// dump_navmesh_source_data_to_mesh_resource(vertices, indices, "test_navmesh_source.tres");

		const RID map_rid = _navigation_region->get_navigation_map();

		NavigationServer3D &ns = *NavigationServer3D::get_singleton();
		const real_t map_cell_size = ns.map_get_cell_size(map_rid);
		const real_t map_cell_height = ns.map_get_cell_height(map_rid);

		Ref<NavigationMesh> navmesh;
		navmesh.instantiate();
		// These must match, so copy them directly...
		navmesh->set_cell_size(map_cell_size);
		navmesh->set_cell_height(map_cell_height);
		// Not using `duplicate` because it is much slower. We could also recycle navmeshes?
		copy_navigation_mesh_settings(**navmesh, **_template_navmesh);

		_baking = true;

		ns.bake_from_source_geometry_data_async(
				navmesh,
				navmesh_source_data,
				callable_mp_static(&VoxelLodTerrainNavigation::on_async_bake_complete_static).bind(get_instance_id(),
	   navmesh)
		);
	*/
}

void VoxelLodTerrainNavigation::on_async_bake_complete_static(uint64_t p_object_id, Ref<NavigationMesh> navmesh) {
	const ObjectID object_id(p_object_id);
	Object *obj = ObjectDB::get_instance(object_id);
	if (obj == nullptr) {
		// Don't raise an error like Callable would. We'll drop the navmesh.
#ifdef DEBUG_ENABLED
		ZN_PRINT_VERBOSE(
				format("Navmesh async baking finished, but {} was destroyed in the meantime.",
					   ZN_CLASS_NAME_C(VoxelLodTerrainNavigation))
		);
#endif
		return;
	}
	VoxelLodTerrainNavigation *nav = Object::cast_to<VoxelLodTerrainNavigation>(obj);
	ZN_ASSERT_RETURN(nav != nullptr);
	nav->on_async_bake_complete(navmesh);
}

void VoxelLodTerrainNavigation::on_async_bake_complete(Ref<NavigationMesh> navmesh) {
	_baking = false;
	//_navigation_region->set_navigation_mesh(navmesh);
}

#ifdef TOOLS_ENABLED

#if defined(ZN_GODOT)
PackedStringArray VoxelLodTerrainNavigation::get_configuration_warnings() const {
	PackedStringArray warnings;
	get_configuration_warnings(warnings);
	return warnings;
}
#elif defined(ZN_GODOT_EXTENSION)
PackedStringArray VoxelLodTerrainNavigation::_get_configuration_warnings() const {
	PackedStringArray warnings;
	get_configuration_warnings(warnings);
	return warnings;
}
#endif

void VoxelLodTerrainNavigation::get_configuration_warnings(PackedStringArray &warnings) const {
	if (_template_navmesh.is_null()) {
		warnings.append("No template is assigned, can't determine settings for baking.");
	}

	if (_template_navmesh.is_valid()) {
		// TODO These warnings can't be updated once the user fixes them.
		// NavigationMesh doesn't emit the `changed` signal when its properties are changed.
		const real_t default_cell_size = 0.25;
		const real_t default_cell_height = 0.25;
		if (!Math::is_equal_approx((double)_template_navmesh->get_cell_size(), (double)default_cell_size)) {
			warnings.append(String("{0} template cell_size has been modified. It won't be used: {1} "
								   "automatically sets "
								   "it to the "
								   "map's cell size when baking.")
									.format(
											varray(NavigationMesh::get_class_static(),
												   VoxelLodTerrainNavigation::get_class_static())
									));
		}
		if (!Math::is_equal_approx((double)_template_navmesh->get_cell_height(), (double)default_cell_height)) {
			warnings.append(String("{0} template cell_height has been modified. It won't be used: {1} "
								   "automatically "
								   "sets it to the "
								   "map's cell height when baking.")
									.format(
											varray(NavigationMesh::get_class_static(),
												   VoxelLodTerrainNavigation::get_class_static())
									));
		}
	}
}

#endif

void VoxelLodTerrainNavigation::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_enabled", "enabled"), &VoxelLodTerrainNavigation::set_enabled);
	ClassDB::bind_method(D_METHOD("is_enabled"), &VoxelLodTerrainNavigation::is_enabled);

	ClassDB::bind_method(D_METHOD("set_run_in_editor", "enabled"), &VoxelLodTerrainNavigation::set_run_in_editor);
	ClassDB::bind_method(D_METHOD("get_run_in_editor"), &VoxelLodTerrainNavigation::get_run_in_editor);

	ClassDB::bind_method(
			D_METHOD("set_template_navigation_mesh", "navmesh"),
			&VoxelLodTerrainNavigation::set_template_navigation_mesh
	);
	ClassDB::bind_method(
			D_METHOD("get_template_navigation_mesh"), &VoxelLodTerrainNavigation::get_template_navigation_mesh
	);

	ClassDB::bind_method(
			D_METHOD("debug_get_regions_visible_in_editor"),
			&VoxelLodTerrainNavigation::debug_get_regions_visible_in_editor
	);
	ClassDB::bind_method(
			D_METHOD("debug_set_regions_visible_in_editor", "debug_regions_visible_in_editor"),
			&VoxelLodTerrainNavigation::debug_set_regions_visible_in_editor
	);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enabled"), "set_enabled", "is_enabled");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "run_in_editor"), "set_run_in_editor", "get_run_in_editor");

	ADD_PROPERTY(
			PropertyInfo(
					Variant::OBJECT,
					"template_navigation_mesh",
					PROPERTY_HINT_RESOURCE_TYPE,
					NavigationMesh::get_class_static(),
					PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT
			),
			"set_template_navigation_mesh",
			"get_template_navigation_mesh"
	);

	ADD_GROUP("Debug", "debug_");

	ADD_PROPERTY(
			PropertyInfo(Variant::BOOL, "debug_regions_visible_in_editor"),
			"debug_set_regions_visible_in_editor",
			"debug_get_regions_visible_in_editor"
	);
}

} // namespace zylann::voxel
