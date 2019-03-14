// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

#pragma once
#include "ModuleManager.h"

THIRD_PARTY_INCLUDES_START
#include <pcl/common/common.h>
#include <pcl/point_types.h>
THIRD_PARTY_INCLUDES_END

#include "Engine.h"
#include "PCLPlugin.h"
#include "PCL_BPLibrary.generated.h"

typedef pcl::PointXYZ PointType;


USTRUCT(BlueprintType)
struct FRange
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Util")
		float Min;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Util")
		float Max;

	float Range() {
		return Max - Min;
	}
};

USTRUCT(BlueprintType)
struct FSensor
{
	GENERATED_USTRUCT_BODY()

	//Posture
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Posture")
		float X;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Posture")
		float Y;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Posture")
		float Z;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Posture")
		float Pitch;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Posture")
		float Yaw;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Posture")
		float Roll;

	//Range
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Sensor Range")
		FRange RangeX;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Sensor Range")
		FRange RangeY;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Sensor Range")
		FRange RangeZ;
};


UCLASS(ClassGroup = PCL, Blueprintable)
class UPCL_BPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_UCLASS_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Sensor")
		FSensor Sensor;


	UFUNCTION(BlueprintPure, meta = (HidePin = "WorldContextObject", DefaultToSelf = "WorldContextObject", DisplayName = "Create PCL Object From Blueprint", CompactNodeTitle = "Create", Keywords = "new create blueprint"), Category = "PCL")
		static UObject* NewObjectFromBlueprint(UObject* WorldContextObject, TSubclassOf<UObject> UC);

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Load PCD File", Keywords = "PCL PCD"), Category = "PCL")
		static TArray<FVector> LoadPCDFile(FString pcdFilePath);

	//PCL Main function
	//*****VLP16
	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Start", Keywords = "VLP16 Start"), Category = "PCL|VLP16")
		void StartVLP(FString IpAddress, int Port);

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Stop", Keywords = "VLP16 Stop"), Category = "PCL|VLP16")
		static void StopVLP();

	//General Input
	UFUNCTION(BlueprintCallable, meta = (DisplayName = "UpdatePoints", Keywords = "Update Depth Points"), Category = "PCL")
		static void UpdatePoints(TArray<FVector> Points);

	//Common
	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Update PointClouds", Keywords = "Update PointCloud Textures"), Category = "PCL")
		void Update();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Input|Filter")
		float InputVoxelSize;

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get Is Enable PointCloud", Keywords = "PCL Get Is Enable"), Category = "PCL")
		bool EnableUpdate();

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get PointCloud Count", Keywords = "PCL Get Num Points"), Category = "PCL")
		int GetNumPoints();


	//RawPointCloud for 
	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get PointCloud", Keywords = "PCL Get PointCloud"), Category = "PCL")
		TArray<FVector> GetPointCloudAsVector();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Interaction|Num Particle")
		int NumMaxPCLActor;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Interaction|Num Particle")
		float MoveRatio;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Interaction|Filter")
		float InteractionVoxelSize;


	//Centorid
	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get Centroids", Keywords = "Get Centroids With FVector"), Category = "PCL")
		TArray<FVector> GetCentroidsAsVector();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Centroid|Filter")
		float CentroidVoxelSize;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Centroid")
		float Tolerance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Centroid")
		int MinRequiredPoint;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Centroid")
		int MaxEnablePoint;

	

	//Generate Texture
	UPROPERTY(BlueprintReadOnly, Category = "PCL|Point Cloud Texture")
		FVector2D TextureSize;

	UPROPERTY(BlueprintReadOnly, Category = "PCL|Point Cloud Texture")
		UTexture2D* PointCloudTexture_Mod;

	UPROPERTY(BlueprintReadOnly, Category = "PCL|Point Cloud Texture")
		UTexture2D* PointCloudTexture_Div;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PCL|Point Cloud Texture|Filter")
		float TextureVoxelSize;

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Update PointCloud Textures", Keywords = "Update PointCloud Textures"), Category = "PCL|Point Cloud Texture")
		void UpdatePointCloudTextures();

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get PointCloud Mod Bytes With Texture", Keywords = "Get PointCloud Mod Bytes With Texture"), Category = "PCL|Point Cloud Texture")
		UTexture2D* GetPointCloudAsTexture_ModBytes();

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get PointCloud Div Bytes With Texture", Keywords = "Get PointCloud Div Bytes With Texture"), Category = "PCL|Point Cloud Texture")
		UTexture2D* GetPointCloudAsTexture_DivBytes();

	
	
private:
	//
	TArray<FColor> PointCloudData_Div;
	TArray<FColor> PointCloudData_Mod;

	//PCL
	pcl::PointCloud<pcl::PointXYZ>::Ptr calcCloud;

	//Filter
	void transformToZeroPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, FSensor& posture, pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud);
	void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, const std::string &fieldName, float min, float max);
	void voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud);
	void statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud);

	//クラスタに分けるやつ
	std::vector<pcl::PointCloud<PointType>::Ptr> eachClouds;
	void calcCentroid();
	void euclideanClusterExtraction(pcl::PointCloud<PointType>::Ptr cloud, std::vector<pcl::PointCloud<PointType>::Ptr> &outputCloud);
	void splitCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr ouputCloud, pcl::PointIndices &indices);
	//重心
	std::vector<Eigen::Vector4f> centroids;
	Eigen::Vector4f centroid(pcl::PointCloud<PointType>::Ptr cloud);
	//z平面にプロジェクションするやつ
	pcl::PointCloud<PointType>::Ptr projectionToZ(pcl::PointCloud<PointType>::Ptr cloud, float zValue);

	//重心のトラッキング
	int numMaxCentroids = 100;
	void calcTrackingCentroids(std::vector<Eigen::Vector4f> &inputCentroids);


	//Actorにするポイントクラウドのトラッキング
	std::vector<pcl::PointXYZ> actorPoints;
	
	void calcTrackingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input);


	//Generate Texture
	void InitDynamicTextureResorces();
	void UpdateTextureRegions(
		UTexture2D* Texture, 
		int32 MipIndex, 
		uint32 NumRegions, 
		FUpdateTextureRegion2D* Regions, 
		uint32 SrcPitch,
		uint32 SrcBpp, 
		uint8* SrcData,
		bool bFreeData);

	//Util
	int NumPointCloud;
	float Clamp(float val, float min, float max);


	

	//pcl_func::Sensor mSensor;
//
//	void Start();
	//void vlpCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloudPtr);
};