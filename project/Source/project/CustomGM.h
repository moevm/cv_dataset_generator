// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BaseCamera.h"
#include "GameFramework/GameModeBase.h"
#include "CustomGM.generated.h"


/**
 * 
 */
UCLASS()
class PROJECT_API ACustomGM : public AGameModeBase
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "CustomGM_Category")
		TArray<ABaseDevice*> GetDeviceArray();

	UFUNCTION(BlueprintCallable, Category = "CustomGM_Category")
		void AddDeviceToArray(ABaseDevice* Device_);
	UFUNCTION(BlueprintCallable, Category = "CustomGM_Category")
		ABaseDevice* FindDeviceByName(FString DeviceName_);
	UFUNCTION(BlueprintCallable, Category = "CustomGM_Category")
		void DeleteDevice(ABaseDevice* Device_);
	UFUNCTION(BlueprintCallable, Category = "CustomGM_Category")
		int64 GetTimeStamp();
	UFUNCTION(BlueprintCallable, Category = "CustomGM_Category")
		void GetDataForDataset(ABaseDevice* Device, FString TimestampStr, int64 TimestampInt, FString ImageName, FString DatasetDir = "DefaultDataset");

private:
	TArray<ABaseDevice*> DeviceArrayAll;
};
