// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "BaseDevice.h"
#include "BaseCamera.generated.h"

/**
 * 
 */
UCLASS()
class PROJECT_API ABaseCamera : public ABaseDevice
{
	GENERATED_BODY()
public:
	ABaseCamera();
	virtual void Tick(float DeltaTime) override;
protected:
	virtual void BeginPlay() override;

	
};
