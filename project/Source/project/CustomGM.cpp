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

void ACustomGM::GetDataForDataset(ABaseDevice* Device, FString TimestampStr, int64 TimestampInt, FString ImageName, FString DatasetDir)
{
	int64 PrevTimestamp = Device->GetPreviousTime();
	FVector Location = Device->GetActorLocation();
	FVector PrevLocation = Device->GetPreviousLocation();
	FQuat Rotation = Device->GetActorRotation().Quaternion();

	int64 DeltaTime = TimestampInt - PrevTimestamp;
	FVector DeltaLocation = Location - PrevLocation;
	FVector Speed = DeltaLocation / DeltaTime;

	FString AxelerationFile = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() + DatasetDir + "/" + Device->GetNameDevice() + TEXT("/axelerometer.txt"));
	FString GrTrFile = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() + DatasetDir + "/" + Device->GetNameDevice() + TEXT("/groundtruth.txt"));
	FString RGBFile = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() + DatasetDir + "/" + Device->GetNameDevice() + TEXT("/rgb.txt"));

	FString SpeedString = TimestampStr + TEXT(" ") + 
		FString::SanitizeFloat(Speed[0]) + TEXT(" ") + 
		FString::SanitizeFloat(Speed[1]) + TEXT(" ") + 
		FString::SanitizeFloat(Speed[2]) + TEXT("\n");
	FFileHelper::SaveStringToFile(SpeedString, *AxelerationFile, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);

	FString GrTrString = TimestampStr + TEXT(" ") +
		FString::SanitizeFloat(Location[0]) + TEXT(" ") +
		FString::SanitizeFloat(Location[1]) + TEXT(" ") +
		FString::SanitizeFloat(Location[2]) + TEXT(" ") +
		FString::SanitizeFloat(Rotation.X) + TEXT(" ") + 
		FString::SanitizeFloat(Rotation.Y) + TEXT(" ") + 
		FString::SanitizeFloat(Rotation.Z) + TEXT(" ") + 
		FString::SanitizeFloat(Rotation.W) + TEXT("\n");
	FFileHelper::SaveStringToFile(GrTrString, *GrTrFile, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);

	FString RGBString = TimestampStr + TEXT(" ") + ImageName + TEXT(".png") + TEXT("\n");
	FFileHelper::SaveStringToFile(RGBString, *RGBFile, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);

	Device->SetPreviousLocation(Location);
	Device->SetPreviousTime(TimestampInt);

}

TArray<ABaseDevice*> ACustomGM::GetDeviceArray()
{
	return this->DeviceArrayAll;
}
