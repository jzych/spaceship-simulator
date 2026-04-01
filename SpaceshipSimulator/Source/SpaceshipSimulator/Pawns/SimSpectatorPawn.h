#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "SimSpectatorPawn.generated.h"

class UCameraComponent;

// Minimal inert pawn used as a camera placeholder.
// All camera logic (position, rotation, zoom) is driven by USimulationSubsystem.
// This pawn intentionally has no movement or input handling of its own.
UCLASS()
class SPACESHIPSIMULATOR_API ASimSpectatorPawn : public APawn
{
    GENERATED_BODY()

public:
    ASimSpectatorPawn();

private:
    UPROPERTY(VisibleAnywhere)
    TObjectPtr<USceneComponent> RootSceneComponent;

    UPROPERTY(VisibleAnywhere)
    TObjectPtr<UCameraComponent> Camera;
};
