#ifndef DIRECT_STATIC_BODY_H
#define DIRECT_STATIC_BODY_H

#include "../non_copyable.h"
#include "classes/shape_3d.h"
#include "direct_mesh_instance.h"
#include "macros.h"

ZN_GODOT_FORWARD_DECLARE(class World3D);

namespace zylann::godot {

// Thin wrapper around static body API
class DirectStaticBody : public zylann::NonCopyable {
public:
	DirectStaticBody();
	~DirectStaticBody();

	void create();
	void destroy();
	bool is_valid() const;
	// True when a shape has actually been attached. Distinct from `is_valid()`, which
	// only says the BODY exists - a body is created before its shape is built, so
	// `is_valid()` answers "will there be a collider" while this answers "is there one".
	inline bool has_shape() const {
		return _shape.is_valid();
	}
	void set_transform(Transform3D transform);
	void add_shape(Ref<Shape3D> shape);
	void remove_shape(int shape_index);
	Ref<Shape3D> get_shape(int shape_index) const;
	void set_world(World3D *world);
	void set_shape_enabled(int shape_index, bool disabled);
	void set_attached_object(const Object *obj);
	void set_collision_layer(int layer);
	void set_collision_mask(int mask);

	void set_debug(bool enabled, World3D *world);

private:
	RID _body;
	Ref<Shape3D> _shape;
	DirectMeshInstance _debug_mesh_instance;
};

} // namespace zylann::godot

#endif // DIRECT_STATIC_BODY_H
