#ifndef VOXEL_LOD_TERRAIN_NAVIGATION_H
#define VOXEL_LOD_TERRAIN_NAVIGATION_H

#include "../../util/godot/classes/node_3d.h"
#include "../../util/godot/macros.h"
#include "../../util/containers/fixed_array.h"
#include "../../util/containers/std_map.h"
#include "../../util/containers/std_unordered_map.h"
#include "../../util/containers/std_vector.h"
#include "../../util/safe_ref_count.h"
#include "../../util/tasks/cancellation_token.h"
#include "../../util/math/vector3i.h"
#include "../../util/thread/mutex.h"

ZN_GODOT_FORWARD_DECLARE(class NavigationRegion3D)
ZN_GODOT_FORWARD_DECLARE(class NavigationMesh)

namespace zylann::voxel {

class VoxelLodTerrain;

// Bakes a single navigation mesh at runtime when terrain loads or changes.
// TODO Rename VoxelLodTerrainNavigationGDSingle
class VoxelLodTerrainNavigation : public Node3D {
	GDCLASS(VoxelLodTerrainNavigation, Node3D)
public:
	VoxelLodTerrainNavigation();
	virtual ~VoxelLodTerrainNavigation();

	void set_enabled(bool p_enabled);
	bool is_enabled() const;

	void set_run_in_editor(bool enable);
	bool get_run_in_editor() const;

	void set_template_navigation_mesh(Ref<NavigationMesh> navmesh);
	Ref<NavigationMesh> get_template_navigation_mesh() const;

	void debug_set_regions_visible_in_editor(bool enable);
	bool debug_get_regions_visible_in_editor() const;

#ifdef TOOLS_ENABLED
#if defined(ZN_GODOT)
	PackedStringArray get_configuration_warnings() const override;
#elif defined(ZN_GODOT_EXTENSION)
	PackedStringArray _get_configuration_warnings() const override;
#endif
	virtual void get_configuration_warnings(PackedStringArray &warnings) const;
#endif

protected:
	void _notification(int p_what);

private:	

	Ref<NavigationMesh> CreateNavMesh();
	NavigationRegion3D* get_available_nav_region();
	void GenerateNavMesh(const VoxelLodTerrain &terrain, Vector3i block_pos);
	void clean_nav_region(Vector3i block);
	void process_nav_meshes(const VoxelLodTerrain& terrain, float delta);
	void update_processing_state();
	void process(float delta_time);
	void bake(const VoxelLodTerrain &terrain);
	static void on_async_bake_complete_static(uint64_t p_object_id, Ref<NavigationMesh> navmesh);
	void on_async_bake_complete(Ref<NavigationMesh> navmesh);

	// Called when an async bake completes for a specific region. This lets us
	// post-process the baked NavigationMesh (for example, clip geometry on the
	// Y axis) before forwarding it to the NavigationRegion3D::_bake_finished
	// handler.
	void on_region_bake_finished(NavigationRegion3D *region, Ref<NavigationMesh> navmesh);

	static void _bind_methods();

	// We only use this for baking settings, polygons in this mesh won't be used
	Ref<NavigationMesh> _template_navmesh;
	
	BinaryMutex nav_region_mutex;
	BinaryMutex nav_queue_mutex;

	std::deque<Vector3i> process_queue;

	struct NavRegionData {
		NavigationRegion3D *region = nullptr;
		uint8_t current_marker = 0;
	};

	std::unordered_map<Vector3i, NavRegionData> _nav_region_active_pool;
	std::vector<NavigationRegion3D*> _available_nav_regions;

	float _time_before_update = 0.f;
	float _update_period_seconds = .2f;
	unsigned int _prev_mesh_updates_count = 0;
	bool _enabled = true;
	bool _baking = false;
	bool _visible_regions_in_editor = false;
	bool _run_in_editor = false;
};

} // namespace zylann::voxel

#endif // VOXEL_LOD_TERRAIN_NAVIGATION_H
