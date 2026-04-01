#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Tickable.h"

THIRD_PARTY_INCLUDES_START
#include "client/render_transform.hpp"
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
//   2. Updates the orbit camera: position tracks Earth, mouse rotates view, scroll zooms
//   3. Interpolates the snapshot buffer at render time
//   4. Updates planet actor positions via camera-relative rendering
//
// Camera: always positioned directly above Earth at CameraAltitudeAU distance.
// Mouse drag rotates the view (yaw + pitch). Scroll wheel zooms in/out.
UCLASS()
class SPACESHIPSIMULATOR_API USimulationSubsystem
    : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    virtual bool IsTickable() const override { return true; }
    virtual bool IsTickableInEditor() const override { return false; }

private:
    // ---------- Simulation state ----------
    TUniquePtr<spaceship::client::SimulationRunner> Runner;

    // ---------- Actor management ----------
    TMap<uint32, TWeakObjectPtr<APlanetActor>> PlanetActors;

    void ReconcilePlanets(
        const std::vector<spaceship::client::InterpolatedMassiveBodyState>& Bodies,
        const spaceship::shared::Vec3& RenderOrigin);

    APlanetActor* SpawnPlanet(uint32 NetId);
    void DestroyAllPlanets();

    // ---------- Orbit camera ----------
    // Camera render origin in simulation coordinates (double, SI metres).
    // Recomputed each tick as EarthPosition + (0, CameraAltitudeAU * AU, 0).
    spaceship::shared::Vec3 RenderOriginSim {};

    // Camera distance above Earth in AU.  Scroll wheel adjusts this.
    double CameraAltitudeAU { kCameraOffsetAU };

    // View direction.  Mouse drag adjusts yaw and pitch each tick.
    FRotator CameraRotation { -90.0, 0.0, 0.0 }; // initial: looking straight down

    // True once the mouse has been captured on the first tick.
    bool bInputInitialized { false };

    // Set to true after the first tick's template-actor cleanup so it only runs once.
    bool bStartupCleanupDone { false };

    void UpdateOrbitCamera(APlayerController* PC, APawn* Pawn);

    // ---------- Coordinate conversion ----------
    static FVector ToUEPosition(
        const spaceship::shared::Vec3& SimPos,
        const spaceship::shared::Vec3& RenderOrigin,
        uint32 NetId);

    static FVector ToUEScale(double RadiusMeters, uint32 NetId);

    // ---------- Constants ----------
    static constexpr double kAstronomicalUnitMeters  = 149597870700.0;
    static constexpr double kCameraOffsetAU          = 0.005;
    static constexpr double kSunDisplayDistanceCm  = 1.0e11;
    static constexpr double kRadiusExaggeration     = 10.0;
    static constexpr float  kMinDisplayScale        = 50.0f;

    // Degrees of view rotation per raw mouse unit (pixel * AxisConfig sensitivity 0.07).
    static constexpr float  kMouseSensitivity       = 1.0f;

    // Zoom factor per scroll notch.  < 1 = zoom-in on scroll-up.
    static constexpr double kScrollZoomFactor       = 0.85;

    static constexpr double kMinAltitudeAU          = 0.0002; //  ~30,000 km (closer than Moon)
    static constexpr double kMaxAltitudeAU          = 2.0;    //  ~2 AU

};
