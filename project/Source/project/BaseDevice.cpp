// Fill out your copyright notice in the Description page of Project Settings.


#include "BaseDevice.h"
#include "Components/StaticMeshComponent.h"

// Sets default values
ABaseDevice::ABaseDevice()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SM_Lamp_Wall"));
	Mesh->SetupAttachment(Root);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> BaseMeshAsset(TEXT("StaticMesh'/Game/StarterContent/Props/SM_Lamp_Wall.SM_Lamp_Wall'"));
	Mesh->SetStaticMesh(BaseMeshAsset.Object);

	this->DeviceName = this->DeviceNameStart;
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

void ABaseDevice::SetIsActive(bool State)
{
	this->isActive = State;
}

bool ABaseDevice::GetIsActive()
{
	return this->isActive;
}

void ABaseDevice::SetDeviceType(FString DeviceType_)
{
	this->DeviceType = DeviceType_;
}

FString ABaseDevice::GetDeviceType()
{
	return this->DeviceType;
}

FVector ABaseDevice::GetPreviousLocation()
{
	return this->PreviousLocation;
}

void ABaseDevice::SetPreviousLocation(FVector Location)
{
	this->PreviousLocation = Location;
}

int64 ABaseDevice::GetPreviousTime()
{
	return this->PreviousTime;
}

void ABaseDevice::SetPreviousTime(int64 Time)
{
	this->PreviousTime = Time;
}

