#pragma once

// Camera-relative coordinate transform for large-world precision.
//
// Converts double-precision SI simulation positions to float-precision
// UE5 centimetre coordinates by subtracting a double-precision render origin
// (the camera's simulation position) before narrowing to float.
//
// Axis remap — Sim (X-fwd, Y-up, Z-right, RH) to UE (X-fwd, Y-right, Z-up, LH):
//   UE.X =  sim.X * 100
//   UE.Y = -sim.Z * 100
//   UE.Z =  sim.Y * 100

#include "shared/sim_types.hpp"
#include "client/client_types.hpp"

namespace spaceship::client
{

// Float-precision UE position in centimetres, relative to the render origin.
struct RenderVec3 { float x{}; float y{}; float z{}; };

// Float-precision UE quaternion (w, x, y, z) with axes remapped to UE conventions.
struct RenderQuat { float w{1.f}; float x{}; float y{}; float z{}; };

// Convert a simulation position to a UE-space centimetre offset from renderOrigin.
// The subtraction is performed in double to preserve precision at astronomical
// distances before narrowing to float.
[[nodiscard]] RenderVec3 to_render_position(
    const shared::Vec3& simPosition,
    const shared::Vec3& renderOrigin);

// Convert a simulation quaternion to a UE-space quaternion.
// The vector part of the quaternion is remapped by the same axis permutation
// used for positions: (w, x, y, z)_sim -> (w, x, -z, y)_ue.
[[nodiscard]] RenderQuat to_render_orientation(
    const shared::Quaternion& simOrientation);

// Choose the render origin for a frame.
//   1. Returns cameraSimPosition if non-zero (camera position is known).
//   2. Falls back to Earth (NetId 1) position if present in state.
//   3. Falls back to world origin {0, 0, 0}.
[[nodiscard]] shared::Vec3 select_render_origin(
    const InterpolatedWorldState& state,
    const shared::Vec3& cameraSimPosition);

} // namespace spaceship::client
