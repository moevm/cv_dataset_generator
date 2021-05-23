// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BaseLidar.generated.h"

UCLASS()
class PROJECT_API ABaseLidar : public AActor
{
	GENERATED_BODY()
	
public:	
	virtual void Tick(float DeltaTime) override;
	ABaseLidar();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	//Simulation settings
	bool bDrawLaserLines = false;
	bool bDrawHitPoints = false;
	bool bIsActive = false;
	bool bInWorldCoordinates = false;
	float TimeStampLidar = 0.0;
	FDelegateHandle OnPostTickDelegate;

	//Lidar settings
	FString LidarName;
	float Range = 1500.0f;

	//Noise settings
	float NoiseStdDev = 0.1;
	float AtmospAttenRate = 0.004f;
};
