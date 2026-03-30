#include "SimulationSubsystem.h"
#include "../Actors/PlanetActor.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "EngineUtils.h"

THIRD_PARTY_INCLUDES_START
#include "server/simulation/simulation_config.hpp"
THIRD_PARTY_INCLUDES_END

DEFINE_LOG_CATEGORY_STATIC(LogSimBridge, Log, All);

// ---------------------------------------------------------------------------
// Subsystem lifecycle
// ---------------------------------------------------------------------------

void USimulationSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    spaceship::server::SimulationConfig simConfig;
    spaceship::client::SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor  = 100000.0;
    runnerConfig.maxTicksPerAdvance = 10000;

    Runner = MakeUnique<spaceship::client::SimulationRunner>(simConfig, runnerConfig);

    UE_LOG(LogSimBridge, Log,
        TEXT("SimulationSubsystem initialized: time scale %.0fx, %d massive bodies"),
        runnerConfig.timeScaleFactor,
        (int)Runner->server().world().massiveBodies.size());

    // Hide default sky/atmosphere/fog actors — irrelevant for a space scene and
    // they produce "skydome doesn't cover scene" warnings at our camera altitude.
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<AActor> It(World); It; ++It)
        {
            const FString ClassName = (*It)->GetClass()->GetName();
            if (ClassName.Contains(TEXT("Sky")) ||
                ClassName.Contains(TEXT("Atmosphere")) ||
                ClassName.Contains(TEXT("Fog")))
            {
                (*It)->SetActorHiddenInGame(true);
                UE_LOG(LogSimBridge, Log, TEXT("Hidden sky actor: %s"), *ClassName);
            }
        }
    }
}

void USimulationSubsystem::Deinitialize()
{
    DestroyAllPlanets();
    Runner.Reset();
    Super::Deinitialize();
}

// ---------------------------------------------------------------------------
// Per-frame tick
// ---------------------------------------------------------------------------

void USimulationSubsystem::Tick(float DeltaTime)
{
    if (!Runner) return;

    Runner->advance(static_cast<double>(DeltaTime));

    const auto& buffer = Runner->snapshotBuffer();
    if (buffer.empty()) return;

    const double latestTime = buffer.latest()->elapsedSeconds;
    const double renderTime = latestTime - spaceship::client::kDefaultInterpolationDelaySeconds;

    const auto state = buffer.interpolate(renderTime);
    if (!state.has_value()) return;

    const auto earthPos = FindEarthPosition(*state);

    ReconcilePlanets(state->massiveBodies, earthPos);
    UpdateCamera(earthPos);
}

TStatId USimulationSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(USimulationSubsystem, STATGROUP_Tickables);
}

// ---------------------------------------------------------------------------
// Actor management
// ---------------------------------------------------------------------------

void USimulationSubsystem::ReconcilePlanets(
    const std::vector<spaceship::client::InterpolatedMassiveBodyState>& Bodies,
    const spaceship::shared::Vec3& RenderOrigin)
{
    TSet<uint32> ActiveIds;
    ActiveIds.Reserve(Bodies.size());

    for (const auto& Body : Bodies)
    {
        const uint32 NetId = Body.netId;
        ActiveIds.Add(NetId);

        TWeakObjectPtr<APlanetActor>* Found = PlanetActors.Find(NetId);
        APlanetActor* Actor = Found ? Found->Get() : nullptr;

        if (!Actor)
        {
            Actor = SpawnPlanet(NetId);
            if (!Actor) continue;
            PlanetActors.Add(NetId, Actor);
        }

        Actor->SetActorLocation(ToUEPosition(Body.position, RenderOrigin, NetId));
        Actor->SetActorScale3D(ToUEScale(Body.radiusMeters, NetId));
    }

    // Despawn actors no longer in the state.
    TArray<uint32> ToRemove;
    for (const auto& Pair : PlanetActors)
    {
        if (!ActiveIds.Contains(Pair.Key))
            ToRemove.Add(Pair.Key);
    }
    for (uint32 Id : ToRemove)
    {
        if (APlanetActor* Actor = PlanetActors[Id].Get())
            Actor->Destroy();
        PlanetActors.Remove(Id);
    }
}

APlanetActor* USimulationSubsystem::SpawnPlanet(uint32 NetId)
{
    UWorld* World = GetWorld();
    if (!World) return nullptr;

    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    APlanetActor* Actor = World->SpawnActor<APlanetActor>(Params);
    if (!Actor) return nullptr;

    Actor->SetPlanetNetId(NetId);

    switch (NetId)
    {
    case 0: Actor->SetDisplayColor(FLinearColor(1.0f, 0.9f, 0.1f)); break; // Sun  — yellow
    case 1: Actor->SetDisplayColor(FLinearColor(0.1f, 0.3f, 1.0f)); break; // Earth — blue
    case 2: Actor->SetDisplayColor(FLinearColor(0.6f, 0.6f, 0.6f)); break; // Moon  — gray
    default: break;
    }

    UE_LOG(LogSimBridge, Log, TEXT("Spawned planet NetId=%u"), NetId);
    return Actor;
}

void USimulationSubsystem::DestroyAllPlanets()
{
    for (auto& Pair : PlanetActors)
    {
        if (APlanetActor* Actor = Pair.Value.Get())
            Actor->Destroy();
    }
    PlanetActors.Empty();
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------

FVector USimulationSubsystem::ToUEPosition(
    const spaceship::shared::Vec3& SimPos,
    const spaceship::shared::Vec3& RenderOrigin,
    uint32 NetId)
{
    // Offset from render origin (Earth) in double — preserves precision.
    const double offX = SimPos.x - RenderOrigin.x;
    const double offY = SimPos.y - RenderOrigin.y;
    const double offZ = SimPos.z - RenderOrigin.z;

    // Sim (X-fwd, Y-up, Z-right, RH)  ->  UE (X-fwd, Y-right, Z-up, LH)
    // UE.X =  offX * 100,  UE.Y = -offZ * 100,  UE.Z = offY * 100
    double ueX = offX * 100.0;
    double ueY = -offZ * 100.0;
    double ueZ = offY * 100.0;

    // Sun (NetId 0) is 1 AU from Earth — float precision breaks down at that
    // distance.  Clamp to kSunDisplayDistanceCm while preserving direction
    // so the Sun appears correctly in the sky.
    if (NetId == 0)
    {
        const double dist = FMath::Sqrt(ueX*ueX + ueY*ueY + ueZ*ueZ);
        if (dist > 0.0)
        {
            const double scale = kSunDisplayDistanceCm / dist;
            ueX *= scale;
            ueY *= scale;
            ueZ *= scale;
        }
    }

    return FVector(
        static_cast<float>(ueX),
        static_cast<float>(ueY),
        static_cast<float>(ueZ));
}

FVector USimulationSubsystem::ToUEScale(double RadiusMeters, uint32 NetId)
{
    // Engine sphere has 50 cm radius.  Scale = radiusCm / 50.
    if (NetId == 0)
    {
        // Sun is clamped to kSunDisplayDistanceCm.  Preserve angular size:
        //   real angular radius = SunRadius / 1 AU  (radians)
        //   display radius (cm) = angular radius * displayDistance * boost
        // Boost of 8 makes the Sun visually prominent (~4° diameter from camera).
        constexpr double kSunRealAngularRadius = 695700000.0 / 149597870700.0; // rad
        constexpr double kSunAngularBoost = 8.0;
        const double displayRadiusCm = kSunRealAngularRadius * kSunDisplayDistanceCm * kSunAngularBoost;
        const float Scale = FMath::Max(
            static_cast<float>(displayRadiusCm / 50.0),
            kMinDisplayScale);
        return FVector(Scale);
    }

    const double radiusCm = RadiusMeters * kRadiusExaggeration * 100.0;
    const float Scale = FMath::Max(
        static_cast<float>(radiusCm / 50.0),
        kMinDisplayScale);
    return FVector(Scale);
}

spaceship::shared::Vec3 USimulationSubsystem::FindEarthPosition(
    const spaceship::client::InterpolatedWorldState& State)
{
    for (const auto& Body : State.massiveBodies)
    {
        if (Body.netId == 1) // Earth
            return Body.position;
    }
    return {0.0, 0.0, 0.0};
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

void USimulationSubsystem::UpdateCamera(const spaceship::shared::Vec3& EarthPos)
{
    UWorld* World = GetWorld();
    if (!World) return;

    APlayerController* PC = World->GetFirstPlayerController();
    if (!PC) return;

    // Camera at 0.005 AU above Earth (sim +Y = UE +Z).
    const float cameraOffsetCm = static_cast<float>(
        kCameraOffsetAU * kAstronomicalUnitMeters * 100.0);

    const FVector CameraLocation(0.0f, 0.0f, cameraOffsetCm);
    const FRotator CameraRotation(-90.0f, 0.0f, 0.0f); // pitch straight down

    if (AActor* Pawn = PC->GetPawn())
    {
        Pawn->SetActorLocation(CameraLocation);
        PC->SetControlRotation(CameraRotation);
    }
    else
    {
        PC->SetControlRotation(CameraRotation);
    }
}
