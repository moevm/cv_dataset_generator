// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "SimGameMode.generated.h"

#include "BaseLidar.h"

/**
 * 
 */
UCLASS()
class PROJECT_API ASimGameMode : public AGameModeBase
{
	GENERATED_BODY()
	virtual void BeginPlay() override;

public:
	UFUNCTION(BlueprintCallable, Category = "Spawn")
		void AddLidarActorToArray(ABaseLidar* LidarActor);

	UFUNCTION(BlueprintCallable, Category = "DeleteActors")
		void DeleteLidar(ABaseLidar* LidarActor);

	//UFUNCTION(BlueprintCallable, Category = "Spawn")
	//	void AddCameraActorToArray(ABaseCamera* CameraActor);

	//UFUNCTION(BlueprintCallable, Category = "DeleteActors")
	//	void DeleteCamera(ABaseCamera* CameraActor);

private:
	TArray<ABaseLidar*> LidarArray;
	//TArray<ABaseCamera*> CameraArray;

	//TArray<ABaseModel*> ModelArray;
	FTimerHandle SimulationTimerHandle;
	float ScanningDurationTime = 10.0f;
	bool bIsDrawLines = false;
	bool bIsDrawPoints = false;
	bool bInWorldCoordinates = false;
	float AtmospAttenRate = 0.004f;
};
