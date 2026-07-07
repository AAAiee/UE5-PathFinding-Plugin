#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/EngineTypes.h"
#include "NavDemoAgent.generated.h"

class AOctNavVolume3D;
class UStaticMeshComponent;

UCLASS()
class TP_THIRDPERSON_API ANavDemoAgent : public AActor
{
	GENERATED_BODY()

public:
	ANavDemoAgent();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	void RequestPath();
	void DrawCurrentPath() const;

private:
	UPROPERTY(VisibleAnywhere, Category = "Nav Demo")
	UStaticMeshComponent* MeshComponent = nullptr;

	UPROPERTY(EditAnywhere, Category = "Nav Demo")
	AOctNavVolume3D* NavVolume = nullptr;

	UPROPERTY(EditAnywhere, Category = "Nav Demo")
	AActor* DestinationActor = nullptr;

	UPROPERTY(EditAnywhere, Category = "Nav Demo", meta = (ClampMin = "1.0"))
	float MovementSpeed = 300.0f;

	UPROPERTY(EditAnywhere, Category = "Nav Demo", meta = (ClampMin = "1.0"))
	float AcceptanceRadius = 25.0f;

	UPROPERTY(EditAnywhere, Category = "Nav Demo", meta = (ClampMin = "1.0"))
	float AgentRadius = 28.0f;

	UPROPERTY(EditAnywhere, Category = "Nav Demo", meta = (ClampMin = "1.0"))
	float AgentHalfHeight = 32.0f;

	UPROPERTY(EditAnywhere, Category = "Nav Demo")
	TArray<TEnumAsByte<EObjectTypeQuery>> ObstacleObjectTypes;

	UPROPERTY(EditAnywhere, Category = "Nav Demo")
	bool bDrawPath = true;

	TArray<FVector> CurrentPath;
	int32 CurrentPathIndex = INDEX_NONE;
};
