#include "SimSpectatorPawn.h"
#include "Camera/CameraComponent.h"

ASimSpectatorPawn::ASimSpectatorPawn()
{
    PrimaryActorTick.bCanEverTick = false;

    bUseControllerRotationPitch = false;
    bUseControllerRotationYaw   = false;
    bUseControllerRotationRoll  = false;

    RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootSceneComponent);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(RootSceneComponent);
    Camera->bUsePawnControlRotation = false;
}
