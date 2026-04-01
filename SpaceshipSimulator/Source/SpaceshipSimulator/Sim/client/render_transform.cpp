#include "client/render_transform.hpp"

namespace spaceship::client
{

RenderVec3 to_render_position(
    const shared::Vec3& simPosition,
    const shared::Vec3& renderOrigin)
{
    // Subtract render origin in double — precision preserved at astronomical distances.
    const double offX = simPosition.x - renderOrigin.x;
    const double offY = simPosition.y - renderOrigin.y;
    const double offZ = simPosition.z - renderOrigin.z;

    // Sim (X-fwd, Y-up, Z-right, RH) -> UE (X-fwd, Y-right, Z-up, LH), metres -> cm.
    return {
        static_cast<float>(offX * 100.0),
        static_cast<float>(-offZ * 100.0),
        static_cast<float>(offY * 100.0)
    };
}

RenderQuat to_render_orientation(const shared::Quaternion& q)
{
    // Apply the same axis permutation to the vector part of the quaternion.
    // sim (w, x, y, z) -> ue (w, x, -z, y)
    return {
        static_cast<float>(q.w),
        static_cast<float>(q.x),
        static_cast<float>(-q.z),
        static_cast<float>(q.y)
    };
}

shared::Vec3 select_render_origin(
    const InterpolatedWorldState& state,
    const shared::Vec3& cameraSimPosition)
{
    // Use a sub-millimetre epsilon — no body legitimately sits at the exact sim origin.
    constexpr double kZeroEpsilonSq = 1e-6; // (1 mm)^2
    const double magSq = cameraSimPosition.x * cameraSimPosition.x
                       + cameraSimPosition.y * cameraSimPosition.y
                       + cameraSimPosition.z * cameraSimPosition.z;
    if (magSq > kZeroEpsilonSq)
        return cameraSimPosition;

    for (const auto& body : state.massiveBodies)
    {
        if (body.netId == 1) // Earth
            return body.position;
    }

    return {0.0, 0.0, 0.0};
}

} // namespace spaceship::client
