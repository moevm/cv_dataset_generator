// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>
#include <chrono>
#include <random>
#include <memory>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BaseDevice.generated.h"

UCLASS()
class PROJECT_API ABaseDevice : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ABaseDevice();
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
		UStaticMeshComponent* Mesh;

	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		FString GetNameDevice();
	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		void SetDeviceName(FString deviceName_);

	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		FString GetPointCloudFileName();
	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		void SetPointCloudFileName(FString PointCloudFileName_);

	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		void SetIsActive(bool State);
	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		bool GetIsActive();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds);

	FString pointCloudFileName;

private:
	bool isActive = false;
	FString DeviceName;
};

