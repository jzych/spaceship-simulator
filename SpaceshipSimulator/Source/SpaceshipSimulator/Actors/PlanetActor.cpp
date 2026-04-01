#include "PlanetActor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/Material.h"
#include "Components/DirectionalLightComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogPlanet, Log, All);

APlanetActor::APlanetActor()
{
    PrimaryActorTick.bCanEverTick = false;

    MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PlanetMesh"));
    RootComponent = MeshComponent;

    // Use engine built-in sphere (radius 50 cm, diameter 100 cm).
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(
        TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    if (SphereMesh.Succeeded())
    {
        MeshComponent->SetStaticMesh(SphereMesh.Object);
    }

    // BasicShapeMaterial supports SetVectorParameterValue("Color") at runtime.
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> BaseMat(
        TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (BaseMat.Succeeded())
    {
        MeshComponent->SetMaterial(0, BaseMat.Object);
    }
    else
    {
        UE_LOG(LogPlanet, Warning, TEXT("BasicShapeMaterial not found — planets will be gray"));
    }

    // Disable all physics and collision — authoritative sim is external.
    MeshComponent->SetSimulatePhysics(false);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    MeshComponent->SetGenerateOverlapEvents(false);
    MeshComponent->SetCastShadow(false);
}

void APlanetActor::SetDisplayColor(const FLinearColor& Color)
{
    if (!MeshComponent) return;

    if (!DynamicMaterial)
    {
        UMaterialInterface* Base = MeshComponent->GetMaterial(0);
        if (!Base)
        {
            UE_LOG(LogPlanet, Warning, TEXT("SetDisplayColor: no material on slot 0"));
            return;
        }

        DynamicMaterial = UMaterialInstanceDynamic::Create(Base, this);
        MeshComponent->SetMaterial(0, DynamicMaterial);

        // Detect which vector parameter name this material exposes and store it
        // so subsequent calls use a single targeted set instead of a blind dual-set.
        // UE5 BasicShapeMaterial has used both "Color" and "BaseColor" across versions.
        TArray<FMaterialParameterInfo> ParamInfos;
        TArray<FGuid> ParamIds;
        DynamicMaterial->GetAllVectorParameterInfo(ParamInfos, ParamIds);
        ColorParamName = FName(TEXT("Color")); // safe default
        for (const FMaterialParameterInfo& Info : ParamInfos)
        {
#if !UE_BUILD_SHIPPING
            UE_LOG(LogPlanet, Log, TEXT("Vector param available: '%s'"), *Info.Name.ToString());
#endif
            if (Info.Name == FName(TEXT("BaseColor")))
                ColorParamName = FName(TEXT("BaseColor"));
        }
    }

    if (DynamicMaterial)
    {
        DynamicMaterial->SetVectorParameterValue(ColorParamName, Color);
    }
}

void APlanetActor::InitSunLighting()
{
    // Switch to an unlit material so the Sun sphere has no dark side.
    UMaterialInterface* UnlitMat = LoadObject<UMaterialInterface>(
        nullptr,
        TEXT("/Engine/EngineDebugMaterials/LevelColorationUnlitMaterial.LevelColorationUnlitMaterial"));
    if (UnlitMat && MeshComponent)
    {
        DynamicMaterial = UMaterialInstanceDynamic::Create(UnlitMat, this);
        MeshComponent->SetMaterial(0, DynamicMaterial);
        UE_LOG(LogPlanet, Log, TEXT("Sun: switched to unlit material"));
    }
    else
    {
        UE_LOG(LogPlanet, Warning, TEXT("Sun: unlit material not found, Sun will have shading"));
    }

    // Add a directional light so Earth and Moon are lit from the Sun's direction.
    // Direction is updated each tick via SetSunLightDirection.
    SunLightComponent = NewObject<UDirectionalLightComponent>(this, TEXT("SunDirectionalLight"));
    SunLightComponent->SetupAttachment(RootComponent);
    SunLightComponent->Intensity   = 10.0f;
    SunLightComponent->LightColor  = FColor(255, 250, 230); // warm white
    SunLightComponent->RegisterComponent();
    UE_LOG(LogPlanet, Log, TEXT("Sun: directional light created"));
}

void APlanetActor::SetSunLightDirection(const FVector& Dir)
{
    if (SunLightComponent)
    {
        SunLightComponent->SetWorldRotation(Dir.Rotation());
    }
}
