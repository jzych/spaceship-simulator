#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SimGameMode.generated.h"

// Minimal game mode: spawns ASimSpectatorPawn for the local player.
UCLASS()
class SPACESHIPSIMULATOR_API ASimGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASimGameMode();
};
