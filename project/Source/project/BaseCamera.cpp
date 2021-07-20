// Fill out your copyright notice in the Description page of Project Settings.


#include "BaseCamera.h"

ABaseCamera::ABaseCamera() : ABaseDevice()
{

}

void ABaseCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ABaseCamera::BeginPlay()
{
	Super::BeginPlay();
}
