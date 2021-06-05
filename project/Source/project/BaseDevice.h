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

	UPROPERTY(EditAnywhere)
		UStaticMeshComponent* Mesh;
	UPROPERTY()
		USceneComponent* Root;
	UPROPERTY(BlueprintReadWrite, Category = "Default", meta = (ExposeOnSpawn = "true"))
		FString DeviceNameStart;

	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		FString GetNameDevice();
	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		void SetDeviceName(FString deviceName_);

	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		void SetIsActive(bool State);
	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		bool GetIsActive();

	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		void SetDeviceType(FString DeviceType_);
	UFUNCTION(BlueprintCallable, Category = "BaseDevice_Category")
		FString GetDeviceType();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	bool isActive = false;
	FString DeviceName;
	FString DeviceType = "None";
};

