#include "SimGameMode.h"
#include "Pawns/SimSpectatorPawn.h"

ASimGameMode::ASimGameMode()
{
    DefaultPawnClass = ASimSpectatorPawn::StaticClass();
}
