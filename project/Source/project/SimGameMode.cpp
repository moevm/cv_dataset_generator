// Fill out your copyright notice in the Description page of Project Settings.


#include "SimGameMode.h"

void ASimGameMode::BeginPlay() {
	Super::BeginPlay();
}

void ASimGameMode::AddLidarActorToArray(ABaseLidar* LidarActor){
	LidarArray.Add(LidarActor);
}

void ASimGameMode::DeleteLidar(ABaseLidar* LidarActor){
	if (LidarActor != nullptr) {
		LidarActor->Destroy();
		LidarArray.Remove(LidarActor);
	}
}

//void ASimGameMode::AddCameraActorToArray(ABaseCamera* CameraActor){
//	CameraArray.Add(CameraActor);
//}

//void ASimGameMode::DeleteCamera(ABaseCamera* CameraActor){
//	if (CameraActor != nullptr) {
//		CameraActor->Destroy();
//		CameraArray.Remove(CameraActor);
//	}
//}
