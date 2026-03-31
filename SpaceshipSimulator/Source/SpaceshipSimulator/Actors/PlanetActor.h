#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PlanetActor.generated.h"

class UDirectionalLightComponent;

// Visual-only actor representing a massive body (Sun, Earth, Moon).
// No UE physics — transforms are set externally by USimulationSubsystem.
UCLASS()
class SPACESHIPSIMULATOR_API APlanetActor : public AActor
{
    GENERATED_BODY()

public:
    APlanetActor();

    void SetPlanetNetId(uint32 InNetId) { PlanetNetId = InNetId; }
    uint32 GetPlanetNetId() const { return PlanetNetId; }

    void SetDisplayColor(const FLinearColor& Color);

    // Call once after spawning the Sun (NetId==0).
    // Switches to an unlit material and adds a directional light to illuminate
    // other bodies from the Sun's direction.
    void InitSunLighting();

    // Update the world direction of the Sun's directional light.
    // Dir should be the unit vector pointing from Sun toward Earth (origin).
    void SetSunLightDirection(const FVector& Dir);

private:
    UPROPERTY(VisibleAnywhere, Category = "Simulation")
    uint32 PlanetNetId = 0;

    UPROPERTY(VisibleAnywhere, Category = "Visual")
    TObjectPtr<UStaticMeshComponent> MeshComponent;

    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> DynamicMaterial;

    // Directional light owned by the Sun actor — null for all non-Sun bodies.
    UPROPERTY()
    TObjectPtr<UDirectionalLightComponent> SunLightComponent;
};
