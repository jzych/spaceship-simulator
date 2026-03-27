# Task: Client Interpolation Buffer (TASK-14)

## Plan

### New Types — `src/client/client_types.hpp`
- `InterpolatedMassiveBodyState`, `InterpolatedShipState`, `InterpolatedProjectileState`
- `InterpolatedWorldState { renderTime, massiveBodies, ships, projectiles }`
- collision events excluded (discrete/non-interpolable)

### Phase 1: Math layer — `snapshot_interpolator.hpp/.cpp`
- `interpolator::lerp(Vec3, Vec3, t)` and `lerp(double, double, t)`
- `interpolator::slerp(Quaternion, Quaternion, t)` — short-arc, fallback to lerp+normalize at dot>0.9995
- `interpolator::interpolateWorldState(s0, s1, renderTime)` — entity matching by netId; spawns use s1 state, despawns are dropped

### Phase 2: Ring buffer — rewrite `client_snapshot_buffer.hpp/.cpp`
- Constructor: `explicit ClientSnapshotBuffer(std::size_t capacity = 64)`
- `push(WorldSnapshot)` — discard if ≤ newest time; overwrite oldest when full
- `interpolate(double renderTime) -> std::optional<InterpolatedWorldState>` — nullopt if out-of-range or empty
- `latest()` — backward-compat, returns `const std::optional<WorldSnapshot>&`
- `size()`, `empty()`, `capacity()`
- Storage: `std::vector<WorldSnapshot>` as circular buffer, `head_` + `count_`

### Phase 3: Tests
- `client_snapshot_buffer_test.cpp` — keep 4 existing, add 10 interpolation tests
- `snapshot_interpolator_test.cpp` — new, slerp/lerp math unit tests (3+)

### Phase 4: CMakeLists.txt
- Add `snapshot_interpolator.cpp` to `spaceship-client-stub`
- Add `snapshot_interpolator_test.cpp` to test executable

## Progress

- [x] client_types.hpp
- [x] snapshot_interpolator.hpp + .cpp
- [x] snapshot_interpolator_test.cpp (RED → GREEN)
- [x] client_snapshot_buffer.hpp rewrite
- [x] client_snapshot_buffer.cpp rewrite
- [x] client_snapshot_buffer_test.cpp (kept 4 existing + 13 new)
- [x] CMakeLists.txt
- [x] Build + all tests green (236/236)

## Review
(to be filled after implementation)
