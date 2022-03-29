// Fill out your copyright notice in the Description page of Project Settings.


#include "CustomGM.h"

void ACustomGM::AddDeviceToArray(ABaseDevice* Device_)
{
	this->DeviceArrayAll.Add(Device_);
}

ABaseDevice* ACustomGM::FindDeviceByName(FString DeviceName_)
{
	for (int i = 0; i < this->DeviceArrayAll.Num(); i++)
	{
		if ((this->DeviceArrayAll[i]->GetNameDevice()).Compare(DeviceName_) == 0)
		{
			return this->DeviceArrayAll[i];
		}
	}

	return nullptr;
}

void ACustomGM::DeleteDevice(ABaseDevice* Device_)
{
	if (Device_ != nullptr)
	{
		Device_->Destroy();
		this->DeviceArrayAll.Remove(Device_);
	}
}

int64 ACustomGM::GetTimeStamp()
{
	FDateTime Time = FDateTime::Now();
	int64 Timestamp = Time.ToUnixTimestamp();
	return Timestamp;
}

TArray<ABaseDevice*> ACustomGM::GetDeviceArray()
{
	return this->DeviceArrayAll;
}
