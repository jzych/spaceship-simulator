#include "PlanetActor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/Material.h"

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

        // Log available vector parameters to help diagnose wrong parameter names.
        TArray<FMaterialParameterInfo> ParamInfos;
        TArray<FGuid> ParamIds;
        DynamicMaterial->GetAllVectorParameterInfo(ParamInfos, ParamIds);
        for (const FMaterialParameterInfo& Info : ParamInfos)
        {
            UE_LOG(LogPlanet, Log, TEXT("Vector param available: '%s'"), *Info.Name.ToString());
        }
    }

    if (DynamicMaterial)
    {
        // Try both known parameter names — UE5 versions vary between "Color" and "BaseColor".
        DynamicMaterial->SetVectorParameterValue(TEXT("Color"), Color);
        DynamicMaterial->SetVectorParameterValue(TEXT("BaseColor"), Color);
    }
}
