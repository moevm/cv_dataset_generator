// Fill out your copyright notice in the Description page of Project Settings.


#include "BaseDevice.h"

#include "Math/Vector.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "DrawDebugHelpers.h"

// Sets default values
ABaseDevice::ABaseDevice()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>("Mesh");
	RootComponent = Mesh;

	DeviceName = GetName();
	pointCloudFileName = DeviceName + "_" + FDateTime::Now().ToString() + ".csv";
	PointCloudWriter = std::make_unique<FileWriter>();

}

// Called when the game starts or when spawned
void ABaseDevice::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void ABaseDevice::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

FString ABaseDevice::GetNameDevice()
{
	return this->DeviceName;
}

void ABaseDevice::SetDeviceName(FString deviceName_)
{
	this->DeviceName = deviceName_;
}

FString ABaseDevice::GetPointCloudFileName()
{
	return this->pointCloudFileName;
}

void ABaseDevice::SetPointCloudFileName(FString PointCloudFileName_)
{
	this->pointCloudFileName = this->DeviceName + "_" + FDateTime::Now().ToString() + ".csv";
}

void ABaseDevice::SetIsActive(bool State)
{
	this->isActive = State;
}

bool ABaseDevice::GetIsActive()
{
	return this->isActive;
}

