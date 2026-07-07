#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NavDemoDestinationMarker.generated.h"

class UStaticMeshComponent;

UCLASS()
class TP_THIRDPERSON_API ANavDemoDestinationMarker : public AActor
{
	GENERATED_BODY()

public:
	ANavDemoDestinationMarker();

private:
	UPROPERTY(VisibleAnywhere, Category = "Nav Demo")
	UStaticMeshComponent* MeshComponent = nullptr;
};
