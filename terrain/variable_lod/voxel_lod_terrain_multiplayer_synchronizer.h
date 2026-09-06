#ifndef VOXEL_NETWORK_LOD_TERRAIN_SYNC_H
#define VOXEL_NETWORK_LOD_TERRAIN_SYNC_H

#include "../../storage/voxel_data_block.h"
#include "../../util/containers/std_unordered_map.h"
#include "../../util/containers/std_vector.h"
#include "../../util/godot/classes/node.h"
#include "../../util/math/box3i.h"

#ifdef TOOLS_ENABLED
#include "../../util/godot/core/version.h"
#endif

namespace zylann::voxel {

class VoxelLodTerrain;
class VoxelBuffer;

// Implements multiplayer replication for `VoxelLodTerrain`.
// The server is authoritative. Data blocks entering the area of remote viewers (VoxelViewers with a
// `network_peer_id` and `requires_data_block_notifications` enabled) are sent to the corresponding peers, at every
// LOD. Edits done on the server are also replicated: either as voxel areas at LOD0 for clients able to update their
// own LODs locally, or as re-sent data blocks at higher LODs for clients too far away to do so.
// Only the Clipbox streaming system is supported, with streaming enabled (no full-load mode).
class VoxelLodTerrainMultiplayerSynchronizer : public Node {
	GDCLASS(VoxelLodTerrainMultiplayerSynchronizer, Node)
public:
	VoxelLodTerrainMultiplayerSynchronizer();

	bool is_server() const;

	// Queues a data block for sending to the given peer. `voxels` may be null, in which case the block is sent as an
	// "empty" block (no edits, can be generated locally by the client if a generator is present).
	void send_block(int viewer_peer_id, const VoxelBuffer *voxels, Vector3i bpos, unsigned int lod_index);
	// Queues a box of voxels at LOD0 to be sent to every remote viewer having it in range.
	// Boxes queued during the same frame get merged when that is cheaper than sending them separately, and are sent
	// at the end of the frame. This matters a lot for continuous editing (a "brush" applied every frame): without it,
	// every single application would serialize and send its own area.
	void send_area(Box3i voxel_box);

	// Tells which peer requested the edits that are about to be applied, so they don't get sent back to it. Set it
	// to 0 again once done. Only useful when clients predict their own edits: echoing an edit back to its author
	// undoes anything it predicted in the meantime, causing a visible stutter during continuous editing.
	void set_edit_source_peer_id(int peer_id);
	int get_edit_source_peer_id() const;

#ifdef TOOLS_ENABLED
#if defined(ZN_GODOT)
	PackedStringArray get_configuration_warnings() const override;
#elif defined(ZN_GODOT_EXTENSION)
	PackedStringArray _get_configuration_warnings() const override;
#endif
	void get_configuration_warnings(PackedStringArray &warnings) const;
#endif

private:
	void _notification(int p_what);

	void process();

	void send_area_immediate(Box3i voxel_box, int excluded_peer_id);
	void flush_pending_areas();

	// Beyond this, merging two edited boxes into one message is not worth the extra voxels it would send
	static constexpr uint64_t MAX_MERGED_AREA_VOLUME = 64 * 64 * 64;

	void _b_receive_blocks(PackedByteArray message_data);
	void _b_receive_area(PackedByteArray message_data);
	void _b_request_blocks(PackedByteArray message_data);
	void _b_force_resync_area(Vector3i origin, Vector3i size);

	// Client side: asks the server for data blocks local viewers want and don't have. See the "block repair" note on
	// `VoxelLodTerrain`. Runs on a timer with backoff, and does nothing at all when nothing is missing, which is the
	// steady state.
	void process_block_repair_requests();

	static void _bind_methods();

	VoxelLodTerrain *_terrain = nullptr;
	int _rpc_channel = 0;

	struct DeferredBlockMessage {
		PackedByteArray data;
	};

	StdUnorderedMap<int, StdVector<DeferredBlockMessage>> _deferred_block_messages_per_peer;

	struct PendingArea {
		Box3i box;
		// Peer that requested this edit, if any. It already applied the edit locally (prediction), so sending it
		// back would revert whatever it has predicted since, only for it to be re-applied a moment later.
		int source_peer_id;
	};

	// Edited boxes waiting to be merged and sent at the end of the frame
	StdVector<PendingArea> _pending_area_boxes;

	// Set by game code while applying an edit requested by a peer. See `set_edit_source_peer_id`.
	int _edit_source_peer_id = 0;

	// --- Block repair channel ---
	//
	// A repair request names blocks the CLIENT is missing, and is answered only with blocks the server's own viewer
	// for that peer already covers, so the best a hostile client can do is receive what it would have been sent
	// anyway — it cannot use this to probe the world. What is left to bound is COST, hence the budget below: a peer
	// that floods gets its extra messages dropped after an O(1) check, and the work the server will do on its behalf
	// per second is capped at roughly what ordinary streaming for that peer already costs.

	// Entries beyond this in a single request are ignored rather than rejecting the message, so a client that asks
	// for a lot still makes progress instead of stalling. It will ask again for the rest.
	static constexpr unsigned int MAX_REQUESTED_BLOCKS_PER_MESSAGE = 256;
	// Messages arriving faster than this from one peer are dropped unparsed.
	static constexpr uint64_t MIN_REQUEST_INTERVAL_MSEC = 200;
	// Blocks one peer can have re-sent per second, however it spreads its requests.
	static constexpr unsigned int MAX_REPAIRED_BLOCKS_PER_SECOND = 512;

	struct RepairBudget {
		uint64_t last_request_msec = 0;
		uint64_t window_start_msec = 0;
		unsigned int blocks_in_window = 0;
	};
	StdUnorderedMap<int, RepairBudget> _repair_budget_per_peer;
	uint64_t _last_repair_budget_cleanup_msec = 0;

	// Client side timer state. The interval backs off when the hole STOPS SHRINKING, so blocks the server will never
	// send (out of its range, or outside its bounds) cost a message every few seconds rather than several a second
	// forever. Progress must be judged on the total size of the hole, not on how many blocks fit in one request:
	// comparing capped request sizes reads as "no progress" even while the hole is closing steadily.
	static constexpr uint64_t MIN_REPAIR_REQUEST_INTERVAL_MSEC = 500;
	// How far the backoff may go depends on WHAT is missing. A hole at LOD0 is the one that has no collision under
	// it, so it is what drops a player through the world and must not be left waiting seconds; holes only at coarse
	// LODs are a distant-terrain pop and can afford to be patient.
	static constexpr uint64_t MAX_REPAIR_REQUEST_INTERVAL_LOD0_MSEC = 1000;
	static constexpr uint64_t MAX_REPAIR_REQUEST_INTERVAL_MSEC = 4000;
	// Ceiling on how much of the hole is tracked at once. Only bounds the bookkeeping; anything past it is picked up
	// by later ticks as the earlier part is filled.
	static constexpr unsigned int MAX_TRACKED_MISSING_BLOCKS = 4096;
	uint64_t _next_repair_request_msec = 0;
	uint64_t _repair_request_interval_msec = MIN_REPAIR_REQUEST_INTERVAL_MSEC;
	unsigned int _last_missing_block_count = 0;
	// Rotates through the missing set when requests stop helping, so blocks the server refuses cannot permanently
	// crowd out blocks it would answer just because they sort first.
	unsigned int _repair_scan_cursor = 0;
};

} // namespace zylann::voxel

#endif // VOXEL_NETWORK_LOD_TERRAIN_SYNC_H
