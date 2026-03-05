# termin-collision Module Boundary

This document fixes the migration boundary for extracting collision code from `termin`.

## Target Scope

Move to `termin-collision`:

- `cpp/termin/colliders/*`
  - `collider.hpp`
  - `collider_primitive.hpp`
  - `box_collider.hpp`
  - `sphere_collider.hpp`
  - `capsule_collider.hpp`
  - `convex_hull_collider.hpp`
  - `gjk.hpp`
  - `attached_collider.hpp`
  - `union_collider.hpp`
  - `colliders.hpp`
  - `collider_component.hpp`
  - `collider_component.cpp`
- `cpp/termin/collision/*`
  - `bvh.hpp`
  - `contact_manifold.hpp`
  - `collision_world.hpp`
  - `collision_world_scene.cpp`
  - `collision_world_c.hpp`
  - `collision_world_c.cpp`
  - `collision.hpp`
- `cpp/termin/tc_collision_c_api.cpp`
- Native binding units:
  - `cpp/termin/colliders_bindings.cpp` (`_colliders_native`)
  - `cpp/termin/collision_bindings.cpp` (`_collision_native`)
- Collision tests:
  - `cpp/tests/tests_colliders.cpp`
  - `cpp/tests/tests_collision.cpp`
  - `cpp/tests/tests_gjk.cpp`
  - `cpp/tests/tests_quickhull.cpp`
  - `cpp/tests/tests_convex_hull_collider.cpp`

## Valid Dependencies

- `termin-scene` (scene handle, entity/component lifecycle, scene extensions)
- `termin-inspect` (inspect registry and field bindings)
- `termin-graphics` (temporary, due to `TcMesh` in `ColliderComponent`)
- `termin-base` (logging/value utilities transitively used by migrated code)

## Out of Scope For This Extraction

- Render passes and gizmo rendering integration in `termin` (`collider_gizmo_pass.*`)
- Scene render state/mount code (`tc_scene_render_ext.*`)
- Physics module extraction (`physics/*`)

## Compatibility Requirements

- Keep Python import surface stable:
  - `termin.colliders.*`
  - `termin.collision.*`
  - `termin.colliders.collider_component.ColliderComponent`
- Keep C API symbols used by `core_c` collision bridge compatible.
