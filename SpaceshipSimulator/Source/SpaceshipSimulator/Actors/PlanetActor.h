#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PlanetActor.generated.h"

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

private:
    UPROPERTY(VisibleAnywhere, Category = "Simulation")
    uint32 PlanetNetId = 0;

    UPROPERTY(VisibleAnywhere, Category = "Visual")
    TObjectPtr<UStaticMeshComponent> MeshComponent;

    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> DynamicMaterial;
};
