// Fill out your copyright notice in the Description page of Project Settings.


#include "BaseCamera.h"

ABaseCamera::ABaseCamera() : ABaseDevice()
{
	this->SetDeviceType("BaseCamera");
}

void ABaseCamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void ABaseCamera::SetCameraActorReference(ACameraActor* Camera_)
{
	this->CameraActorReference = Camera_;
}

ACameraActor* ABaseCamera::GetCameraActorReference()
{
	return this->CameraActorReference;
}

void ABaseCamera::BeginPlay()
{
	Super::BeginPlay();
}
