#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Tickable.h"

// .generated.h must be the last #include — sim headers go before it.
THIRD_PARTY_INCLUDES_START
#include "client/simulation_runner.hpp"
#include "client/client_types.hpp"
#include "shared/sim_types.hpp"
THIRD_PARTY_INCLUDES_END

#include "SimulationSubsystem.generated.h"

class APlanetActor;

// Drives the simulation and updates UE5 actors each frame.
//
// Owns a SimulationRunner (which owns SimulationServer + ClientSnapshotBuffer).
// Each Tick:
//   1. Advances the simulation by scaled wall-clock time
//   2. Interpolates the snapshot buffer at render time
//   3. Updates planet actor positions using Earth-relative coordinates
//
// Camera follows Earth at a configurable distance so the Moon orbit is visible.
UCLASS()
class SPACESHIPCLIENT_API USimulationSubsystem
    : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    // USubsystem interface
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    // FTickableGameObject interface
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    virtual bool IsTickable() const override { return true; }
    virtual bool IsTickableInEditor() const override { return false; }

private:
    // ---------- Simulation state ----------
    TUniquePtr<spaceship::client::SimulationRunner> Runner;

    // ---------- Actor management ----------
    // Maps NetId -> spawned planet actor.
    TMap<uint32, TWeakObjectPtr<APlanetActor>> PlanetActors;

    void ReconcilePlanets(
        const std::vector<spaceship::client::InterpolatedMassiveBodyState>& Bodies,
        const spaceship::shared::Vec3& RenderOrigin);

    APlanetActor* SpawnPlanet(uint32 NetId);
    void DestroyAllPlanets();

    // ---------- Coordinate conversion ----------
    // Camera-relative: subtract RenderOrigin in double, scale m->cm, remap axes,
    // then narrow to float.
    //   Sim:  X-forward, Y-up,    Z-right  (right-handed)
    //   UE5:  X-forward, Y-right, Z-up     (left-handed)
    //   Mapping: UE(x, y, z) = Sim(x, -z, y) * 100
    // NetId is used to apply Sun-specific distance clamping.
    static FVector ToUEPosition(
        const spaceship::shared::Vec3& SimPos,
        const spaceship::shared::Vec3& RenderOrigin,
        uint32 NetId);

    // Convert simulation radius (meters) to UE scale factor.
    // Engine sphere has 50 cm radius; scale = radiusCm / 50.
    // Applies a minimum display scale so tiny bodies remain visible.
    // NetId is used to apply Sun-specific radius exaggeration.
    static FVector ToUEScale(double RadiusMeters, uint32 NetId);

    // Find Earth's position in the interpolated state (NetId 1).
    static spaceship::shared::Vec3 FindEarthPosition(
        const spaceship::client::InterpolatedWorldState& State);

    // ---------- Camera ----------
    void UpdateCamera(const spaceship::shared::Vec3& EarthPos);

    // Camera offset from Earth in sim coordinates (meters).
    // 0.005 AU along +Y (sim up) so we look down at the orbital plane.
    static constexpr double kCameraOffsetAU = 0.005;
    static constexpr double kAstronomicalUnitMeters = 149597870700.0;

    // Sun is 1 AU away — float precision breaks at that distance.
    // Clamp Sun display position to this distance (cm) while preserving direction.
    // 1e11 cm = 1,000,000 km (> camera altitude of ~750,000 km, so Sun appears behind Earth).
    static constexpr double kSunDisplayDistanceCm = 1.0e11;

    // Visual radius exaggeration for prototype (real radii are invisible at
    // 0.005 AU viewing distance).
    static constexpr double kRadiusExaggeration = 10.0;

    // Minimum display scale (UE scale units) so no planet is invisible.
    static constexpr float kMinDisplayScale = 50.0f;
};
