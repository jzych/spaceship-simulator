#include "SimulationSubsystem.h"
#include "Actors/PlanetActor.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/PlayerInput.h"
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
    runnerConfig.timeScaleFactor    = 100000.0;
    runnerConfig.maxTicksPerAdvance = 10000;

    Runner = MakeUnique<spaceship::client::SimulationRunner>(simConfig, runnerConfig);

    UE_LOG(LogSimBridge, Log,
        TEXT("SimulationSubsystem initialized: time scale %.0fx, %d massive bodies"),
        runnerConfig.timeScaleFactor,
        (int)Runner->server().world().massiveBodies.size());

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

    // Destroy default template actors (sky, landscape, fog, clouds, etc.) during
    // startup. Runs every tick for kStartupCleanupDuration seconds so World
    // Partition streaming actors that arrive after the first tick are also caught.
    if (StartupCleanupTimer > 0.0f)
    {
        StartupCleanupTimer -= DeltaTime;
        if (UWorld* World = GetWorld())
        {
            TArray<AActor*> ToDestroy;
            for (TActorIterator<AActor> It(World); It; ++It)
            {
                const FString ClassName = (*It)->GetClass()->GetName();
                if (ClassName.Contains(TEXT("Landscape"))        ||
                    ClassName.Contains(TEXT("Sky"))              ||
                    ClassName.Contains(TEXT("Atmosphere"))       ||
                    ClassName.Contains(TEXT("Fog"))              ||
                    ClassName.Contains(TEXT("Cloud"))            ||
                    ClassName.Contains(TEXT("DirectionalLight")) ||
                    ClassName.Equals(TEXT("StaticMeshActor"))    ||
                    ClassName.Equals(TEXT("Brush")))
                {
                    ToDestroy.Add(*It);
                }
            }
            for (AActor* Actor : ToDestroy)
            {
                UE_LOG(LogSimBridge, Log, TEXT("Destroyed template actor: %s (%s)"),
                    *Actor->GetName(), *Actor->GetClass()->GetName());
                Actor->Destroy();
            }
        }
    }

    Runner->advance(static_cast<double>(DeltaTime));

    const auto& buffer = Runner->snapshotBuffer();
    if (buffer.empty()) return;

    const double latestTime = buffer.latest()->elapsedSeconds;
    const double renderTime = latestTime - spaceship::client::kDefaultInterpolationDelaySeconds;

    const auto state = buffer.interpolate(renderTime);
    if (!state.has_value()) return;

    // Camera position = Earth + altitude in sim Y (up direction).
    // Recomputed every tick so the camera tracks Earth's orbital motion.
    RenderOriginSim = spaceship::client::select_render_origin(*state, {0.0, 0.0, 0.0});
    RenderOriginSim.y += CameraAltitudeAU * kAstronomicalUnitMeters;

    // Drive orbit camera from player input.
    if (UWorld* World = GetWorld())
    {
        if (APlayerController* PC = World->GetFirstPlayerController())
        {
            if (APawn* Pawn = PC->GetPawn())
                UpdateOrbitCamera(PC, Pawn);
        }
    }

    ReconcilePlanets(state->massiveBodies, RenderOriginSim);
}

TStatId USimulationSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(USimulationSubsystem, STATGROUP_Tickables);
}

// ---------------------------------------------------------------------------
// Orbit camera
// ---------------------------------------------------------------------------

void USimulationSubsystem::UpdateOrbitCamera(APlayerController* PC, APawn* Pawn)
{
    // Capture mouse once on the first tick so look works immediately.
    if (!bInputInitialized)
    {
        PC->SetInputMode(FInputModeGameOnly());
        PC->SetShowMouseCursor(false);
        bInputInitialized = true;
    }

    if (PC->PlayerInput)
    {
        // --- Mouse look (yaw + pitch) ---
        const float MouseX = PC->PlayerInput->GetKeyValue(EKeys::MouseX);
        const float MouseY = PC->PlayerInput->GetKeyValue(EKeys::MouseY);

        CameraRotation.Yaw  += MouseX * kMouseSensitivity;
        // UE convention: mouse up = positive MouseY; pitch positive = nose down.
        CameraRotation.Pitch = FMath::ClampAngle(
            CameraRotation.Pitch + MouseY * kMouseSensitivity, -89.f, 89.f);
        CameraRotation.Roll  = 0.f;

        // --- Scroll zoom ---
        const float ScrollUp   = PC->PlayerInput->GetKeyValue(EKeys::MouseScrollUp);
        const float ScrollDown = PC->PlayerInput->GetKeyValue(EKeys::MouseScrollDown);
        if (ScrollUp   > 0.f) CameraAltitudeAU *= kScrollZoomFactor;
        if (ScrollDown > 0.f) CameraAltitudeAU /= kScrollZoomFactor;
        CameraAltitudeAU = FMath::Clamp(CameraAltitudeAU, kMinAltitudeAU, kMaxAltitudeAU);
    }

    // Pawn is the camera: pin it at (0,0,0) in UE space (= the render origin)
    // and apply the current view rotation.
    Pawn->SetActorLocation(FVector::ZeroVector);
    Pawn->SetActorRotation(CameraRotation);
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

        const FVector UEPos = ToUEPosition(Body.position, RenderOrigin, NetId);
        Actor->SetActorLocation(UEPos);
        Actor->SetActorScale3D(ToUEScale(Body.radiusMeters, NetId));

        // Keep the Sun's directional light aimed from the Sun toward Earth (origin).
        if (NetId == 0 && !UEPos.IsNearlyZero())
            Actor->SetSunLightDirection((-UEPos).GetSafeNormal());
    }

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
    case 0:
        Actor->SetDisplayColor(FLinearColor(1.0f, 0.9f, 0.1f)); // Sun — yellow
        Actor->InitSunLighting();
        break;
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
    if (NetId == 0)
    {
        // Sun is ~1 AU away — compute direction in double then clamp distance.
        const double offX = SimPos.x - RenderOrigin.x;
        const double offY = SimPos.y - RenderOrigin.y;
        const double offZ = SimPos.z - RenderOrigin.z;

        double ueX =  offX * 100.0;
        double ueY = -offZ * 100.0;
        double ueZ =  offY * 100.0;

        const double dist = FMath::Sqrt(ueX*ueX + ueY*ueY + ueZ*ueZ);
        if (dist > 0.0)
        {
            const double scale = kSunDisplayDistanceCm / dist;
            ueX *= scale; ueY *= scale; ueZ *= scale;
        }
        return FVector(ueX, ueY, ueZ);
    }

    const spaceship::client::RenderVec3 pos =
        spaceship::client::to_render_position(SimPos, RenderOrigin);
    return FVector(pos.x, pos.y, pos.z);
}

FVector USimulationSubsystem::ToUEScale(double RadiusMeters, uint32 NetId)
{
    if (NetId == 0)
    {
        constexpr double kSunRealAngularRadius = 695'700'000.0 / 149'597'870'700.0;
        constexpr double kSunAngularBoost      = 8.0;
        const double displayRadiusCm =
            kSunRealAngularRadius * kSunDisplayDistanceCm * kSunAngularBoost;
        const float Scale = FMath::Max(
            static_cast<float>(displayRadiusCm / 50.0), kMinDisplayScale);
        return FVector(Scale);
    }

    const double radiusCm = RadiusMeters * kRadiusExaggeration * 100.0;
    const float Scale = FMath::Max(
        static_cast<float>(radiusCm / 50.0), kMinDisplayScale);
    return FVector(Scale);
}
