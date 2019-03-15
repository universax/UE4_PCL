// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.
#include "PCL_BPLibrary.h"

#pragma region Disable Waring C4996
//
// Disable Warning C4996
//
#ifndef _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif
#ifndef _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES 1
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif
#pragma endregion

#include <random>
#include <functional>
#include <ctime>
#include <thread>

THIRD_PARTY_INCLUDES_START
//IO
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
//Filter
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
//Surface
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
//Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//ROS
#include <pcl/ros/conversions.h>
//Registration
#include <pcl/registration/icp.h>
//KD Tree
#include <pcl/kdtree/kdtree_flann.h>
//Feature
#include <pcl/features/normal_3d.h>
//Image
#include <pcl/range_image/range_image.h>
//VLP Grabber
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
THIRD_PARTY_INCLUDES_END

DEFINE_LOG_CATEGORY(PCLPlugin);


//--------------------------------------------------------------------------------------
//	Vars
//--------------------------------------------------------------------------------------
boost::shared_ptr<pcl::VLPGrabber> mVlpGrabber;
pcl::PointCloud<pcl::PointXYZI>::ConstPtr mVLPCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr mInputCloud;
pcl::PointCloud<pcl::PointXYZ> pcdCloud;
boost::signals2::connection mConnection;
boost::mutex mVLPMutex;
bool IsEnableSensor;

//DynamnicTexture
FUpdateTextureRegion2D* PointCloudUpdateTextureRegion;
bool DidDynamicTextureInit;



//--------------------------------------------------------------------------------------
//	Initializer
//--------------------------------------------------------------------------------------
UPCL_BPLibrary::UPCL_BPLibrary(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	UE_LOG(PCLPlugin, Warning, TEXT("UPCL_BPLibrary Constructor"));
	DidDynamicTextureInit = false;
}

//Blueprintからインスタンス化するためのファンクション
UObject* UPCL_BPLibrary::NewObjectFromBlueprint(UObject* WorldContextObject, TSubclassOf<UObject> UC)
{
	UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject);
	
	
	return NewObject<UPCL_BPLibrary>(UC);
}



//--------------------------------------------------------------------------------------
//	For Test
//--------------------------------------------------------------------------------------
TArray<FVector> UPCL_BPLibrary::LoadPCDFile(FString pcdFilePath) {

	std::string filePath = std::string(TCHAR_TO_UTF8(*pcdFilePath));
	TArray<FVector> cloudPoints;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, pcdCloud) == -1)
	{
		UE_LOG(PCLPlugin, Warning, TEXT("Couldn't read file test_pcd.pcd \n"));
		return cloudPoints;
	}

	auto w = FString::FromInt(pcdCloud.width * pcdCloud.height);
	UE_LOG(PCLPlugin, Warning, TEXT("Succesfully loaded %s data points."), *w);
	for (auto p :pcdCloud.points)
	{
		FVector v;
		v.X = p.x;
		v.Y = p.y;
		v.Z = p.z;

		cloudPoints.Add(v);
	}

	return cloudPoints;
}


//--------------------------------------------------------------------------------------
//	VLP16 Basic
//--------------------------------------------------------------------------------------
//Callback
void vlpCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloudPtr)
{
	mVLPMutex.lock();
	mVLPCloud = cloudPtr;
	mVLPMutex.unlock();

	IsEnableSensor = true;
	//UE_LOG(PCLPlugin, Warning, TEXT("YEEEAAAAA"));

}

void UPCL_BPLibrary::StartVLP(FString IpAddress, int Port) {
	if (mVlpGrabber)
	{
		return;
	}

	
	std::string addr = std::string(TCHAR_TO_UTF8(*IpAddress));
	unsigned short port = Port;

	UE_LOG(PCLPlugin, Warning, TEXT("Start loading VLP16 at IP: %s, PORT: %d"), addr.c_str(), Port);
	mVlpGrabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(boost::asio::ip::address::from_string(addr), boost::lexical_cast<unsigned short>(port)));

	boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> cb = boost::bind(&vlpCallback, _1);
	mConnection = mVlpGrabber->registerCallback(cb);
	mVlpGrabber->start();
}

void UPCL_BPLibrary::StopVLP() {
	if (!mVlpGrabber)
	{
		return;
	}

	UE_LOG(PCLPlugin, Warning, TEXT("Stopping VLP16"));
	//
	//UE_LOG(PCLPlugin, Warning, TEXT("1"));
	mConnection.disconnect();
	if (mVlpGrabber->isRunning())
	{
		mVlpGrabber->stop();
		UE_LOG(PCLPlugin, Warning, TEXT("Hahahaha"));
	}
	//UE_LOG(PCLPlugin, Warning, TEXT("2"));
	mVlpGrabber = nullptr;

	return;

	if (mVlpGrabber->isRunning()) {
		UE_LOG(PCLPlugin, Warning, TEXT("Stop Grabber"));
		mVlpGrabber->stop();
	}
	if (mConnection.connected())
	{
		UE_LOG(PCLPlugin, Warning, TEXT("Stop Connection"));
		mConnection.disconnect();
	}
}


//--------------------------------------------------------------------------------------
//	General Depth Sensor Input
//--------------------------------------------------------------------------------------
void UPCL_BPLibrary::UpdatePoints(TArray<FVector> Points)
{
	mInputCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	for (auto &p : Points)
	{
		pcl::PointXYZ point;
		point.x = p.X;
		point.y = p.Y;
		point.z = p.Z;

		mInputCloud->points.push_back(point);
	}
	if (!mInputCloud)
	{
		return;
	}

	if (mInputCloud->points.size() > 0)
	{
		IsEnableSensor = true;
		calcCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*mInputCloud, *calcCloud);

		//PointSize
		NumPointCloud = calcCloud->points.size();
	}
}



//--------------------------------------------------------------------------------------
//	Common
//--------------------------------------------------------------------------------------
TArray<FVector> UPCL_BPLibrary::GetPointCloudAsVector() {
	//Copy to FVector
	TArray<FVector> cloudPoints;
	if (!calcCloud)
	{
		return cloudPoints;
	}

	//Filtere
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*calcCloud, *tmpCloud);

	//トラッキングする
	voxelGridFilter(InteractionVoxelSize, tmpCloud);
	calcTrackingPoints(tmpCloud);

	//FVectorにして渡す
	for (auto p : actorPoints)
	{
		FVector v;
		v.X = p.x;
		v.Y = p.y;
		v.Z = p.z;

		cloudPoints.Add(v);
	}

	return cloudPoints;
}

TArray<FVector> UPCL_BPLibrary::GetCentroidsAsVector() {
	calcCentroid();

	TArray<FVector> centroidPoints;
	if (!calcCloud)
	{
		return centroidPoints;
	}

	for (auto p : centroids)
	{
		FVector v;
		v.X = p.x();
		v.Y = p.y();
		v.Z = p.z();
		
		centroidPoints.Add(v);
	}

	return centroidPoints;
}



bool UPCL_BPLibrary::EnableUpdate()
{
	return IsEnableSensor;
}

int UPCL_BPLibrary::GetNumPoints()
{
	return NumPointCloud;
}

FVector UPCL_BPLibrary::GetInputRange()
{
	FVector val;
	val.X = minInputVal;
	val.Y = maxInputVal;
	return val;
}

//--------------------------------------------------------------------------------------
//	Dynamic Texture
//--------------------------------------------------------------------------------------
//Texture生成周りの処理
void UPCL_BPLibrary::InitDynamicTextureResorces() {
	TextureSize.X = 256;
	TextureSize.Y = 256;
#undef UpdateResource	//winbase.hのマクロ定義と衝突するので回避
	PointCloudTexture_Div = UTexture2D::CreateTransient(TextureSize.X, TextureSize.Y, PF_B8G8R8A8);
	PointCloudTexture_Div->UpdateResource();
	PointCloudData_Div.Init(FColor(0, 0, 0, 255), TextureSize.X * TextureSize.Y);
	PointCloudTexture_Mod = UTexture2D::CreateTransient(TextureSize.X, TextureSize.Y, PF_B8G8R8A8);
	PointCloudTexture_Mod->UpdateResource();
	PointCloudData_Mod.Init(FColor(0, 0, 0, 255), TextureSize.X * TextureSize.Y);
	
	PointCloudUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, TextureSize.X, TextureSize.Y);

	UE_LOG(PCLPlugin, Warning, TEXT("Init DynamicTexture"));
}

void UPCL_BPLibrary::Update() {

}

void UPCL_BPLibrary::UpdatePointCloudTextures() {
	if (!calcCloud)
	{
		return;
	}
	//初回だけテクスチャ周りをイニシャライズしておく
	if (!DidDynamicTextureInit)
	{
		InitDynamicTextureResorces();
		DidDynamicTextureInit = true;
	}

	//Nullチェック
	if (!PointCloudTexture_Div || !PointCloudTexture_Mod)
	{
		UE_LOG(PCLPlugin, Error, TEXT("ERROR: PointCloudTexture is null"));
		return;
	}

	//Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*calcCloud, *tmpCloud);
	//voxelGridFilter(TextureVoxelSize, tmpCloud);
	//statisticalOutlierFilter(tmpCloud);


	int np = tmpCloud->points.size();
	if (np > 0)
	{
		//ポイントクラウドを最大サイズにリサイズして後ろの空白埋めるなり、余りをクロップスるなりする
		int numEnableMaxPoint = TextureSize.X * TextureSize.Y;

		//Get max and min value
		maxInputVal = tmpCloud->points[0].x;
		minInputVal = tmpCloud->points[0].x;
		for (int i = 0; i < numEnableMaxPoint; i++)
		{
			if (i < np)
			{
				// max
				if (maxInputVal < tmpCloud->points[i].x) maxInputVal = tmpCloud->points[i].x;
				if (maxInputVal < tmpCloud->points[i].y) maxInputVal = tmpCloud->points[i].y;
				if (maxInputVal < tmpCloud->points[i].z) maxInputVal = tmpCloud->points[i].z;

				// min
				if (minInputVal > tmpCloud->points[i].x) minInputVal = tmpCloud->points[i].x;
				if (minInputVal > tmpCloud->points[i].y) minInputVal = tmpCloud->points[i].y;
				if (minInputVal > tmpCloud->points[i].z) minInputVal = tmpCloud->points[i].z;
			}
		}

		for (int i = 0; i < numEnableMaxPoint; i++)
		{
			uint8 x_div = 0;
			uint8 y_div = 0;
			uint8 z_div = 0;

			uint8 x_mod = 0;
			uint8 y_mod = 0;
			uint8 z_mod = 0;

			if (i < np)
			{
				//m -> mm
				float maxVal = 256.0 * 256.0 - 1;
				uint16 x = maxVal * Clamp(tmpCloud->points[i].x, minInputVal, maxInputVal);
				uint16 y = maxVal * Clamp(tmpCloud->points[i].y, minInputVal, maxInputVal);
				uint16 z = maxVal * Clamp(tmpCloud->points[i].z, minInputVal, maxInputVal);

				//商
				x_div = uint8(x / 256);
				y_div = uint8(y / 256);
				z_div = uint8(z / 256);

				//余
				x_mod = x % 256;
				y_mod = y % 256;
				z_mod = z % 256;
			}
			
			PointCloudData_Div[i].R = x_div;	//x
			PointCloudData_Div[i].G = y_div;	//y
			PointCloudData_Div[i].B = z_div;	//z

			PointCloudData_Mod[i].R = x_mod;	//x
			PointCloudData_Mod[i].G = y_mod;	//y
			PointCloudData_Mod[i].B = z_mod;	//z
		}

		UpdateTextureRegions(
			PointCloudTexture_Div,
			(int32)0,
			(uint32)1,
			PointCloudUpdateTextureRegion,
			(uint32)4 * TextureSize.X,
			(uint32)4,
			(uint8*)PointCloudData_Div.GetData(),
			false
		);

		UpdateTextureRegions(
			PointCloudTexture_Mod,
			(int32)0,
			(uint32)1,
			PointCloudUpdateTextureRegion,
			(uint32)4 * TextureSize.X,
			(uint32)4,
			(uint8*)PointCloudData_Mod.GetData(),
			false
		);

		//UE_LOG(PCLPlugin, Warning, TEXT("Update Texture"));
	}
	else {
		IsEnableSensor = false;
		//UE_LOG(PCLPlugin, Error, TEXT("ERROR: PointCloud is null"));
	}
}

UTexture2D * UPCL_BPLibrary::GetPointCloudAsTexture_ModBytes()
{
	if (!PointCloudTexture_Mod)
	{
		UE_LOG(PCLPlugin, Warning, TEXT("ERROR: DynamicTexture_ModBytes is null"));
	}

	return PointCloudTexture_Mod;
}

UTexture2D * UPCL_BPLibrary::GetPointCloudAsTexture_DivBytes()
{
	if (!PointCloudTexture_Div)
	{
		UE_LOG(PCLPlugin, Warning, TEXT("ERROR: DynamicTexture_DivBytes is null"));
	}

	return PointCloudTexture_Div;
}

void UPCL_BPLibrary::UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData)
{
	if (Texture->Resource)
	{
		struct FUpdateTextureRegionsData
		{
			FTexture2DResource* Texture2DResource;
			int32 MipIndex;
			uint32 NumRegions;
			FUpdateTextureRegion2D* Regions;
			uint32 SrcPitch;
			uint32 SrcBpp;
			uint8* SrcData;
		};

		FUpdateTextureRegionsData* RegionData = new FUpdateTextureRegionsData;

		RegionData->Texture2DResource = (FTexture2DResource*)Texture->Resource;
		RegionData->MipIndex = MipIndex;
		RegionData->NumRegions = NumRegions;
		RegionData->Regions = Regions;
		RegionData->SrcPitch = SrcPitch;
		RegionData->SrcBpp = SrcBpp;
		RegionData->SrcData = SrcData;

		ENQUEUE_UNIQUE_RENDER_COMMAND_TWOPARAMETER(
			UpdateTextureRegionsData,
			FUpdateTextureRegionsData*, RegionData, RegionData,
			bool, bFreeData, bFreeData,
			{
				for (uint32 RegionIndex = 0; RegionIndex < RegionData->NumRegions; ++RegionIndex)
				{
					int32 CurrentFirstMip = RegionData->Texture2DResource->GetCurrentFirstMip();
					if (RegionData->MipIndex >= CurrentFirstMip)
					{
						RHIUpdateTexture2D(
							RegionData->Texture2DResource->GetTexture2DRHI(),
							RegionData->MipIndex - CurrentFirstMip,
							RegionData->Regions[RegionIndex],
							RegionData->SrcPitch,
							RegionData->SrcData
							+ RegionData->Regions[RegionIndex].SrcY * RegionData->SrcPitch
							+ RegionData->Regions[RegionIndex].SrcX * RegionData->SrcBpp
						);
					}
				}
		if (bFreeData)
		{
			FMemory::Free(RegionData->Regions);
			FMemory::Free(RegionData->SrcData);
		}
		delete RegionData;
			});
	}
}

void UPCL_BPLibrary::transformToZeroPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, FSensor& posture, pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud)
{
	//RotateX
	float pitchRad = posture.Pitch / 180.0 * M_PI;
	Eigen::Affine3f transformRotateX = Eigen::Affine3f::Identity();
	transformRotateX.rotate(Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*inputCloud, *outputCloud, transformRotateX);

	//RotateY
	float yawRad = posture.Yaw / 180.0 * M_PI;
	Eigen::Affine3f transformRotateY = Eigen::Affine3f::Identity();
	transformRotateY.rotate(Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*outputCloud, *outputCloud, transformRotateY);

	//RotateZ
	float rollRad = posture.Roll / 180.0 * M_PI;
	Eigen::Affine3f transformRotateZ = Eigen::Affine3f::Identity();
	transformRotateZ.rotate(Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*outputCloud, *outputCloud, transformRotateZ);

	//移動
	Eigen::Affine3f transformMove = Eigen::Affine3f::Identity();
	transformMove.translation() << posture.X, posture.Y, posture.Z;
	pcl::transformPointCloud(*outputCloud, *outputCloud, transformMove);
}

void UPCL_BPLibrary::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, const std::string &fieldName, float min, float max) {
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(inputCloud);
	pass.setFilterFieldName(fieldName);
	pass.setFilterLimits(min, max);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filterdCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pass.filter(*filterdCloud);
	*inputCloud = *filterdCloud;
}

void UPCL_BPLibrary::voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud)
{
	//VoxelGrid
	pcl::VoxelGrid<PointType> grid;
	grid.setLeafSize(leaf, leaf, leaf);		//フィルター範囲設定
	grid.setInputCloud(cloud);
	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
	grid.filter(*cloud_filtered);
	pcl::copyPointCloud(*cloud_filtered, *cloud);
	cloud_filtered.reset();
}


void UPCL_BPLibrary::statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud)
{
	pcl::StatisticalOutlierRemoval<PointType> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(5);				//近接何ポイントを使うか
	sor.setStddevMulThresh(1.0);	//この標準偏差以上をフィルターして切る

	pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
	sor.filter(*cloud_filtered);

	pcl::copyPointCloud(*cloud_filtered, *cloud);
}


void UPCL_BPLibrary::calcTrackingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input) {
	if (!input)
	{
		return;
	}
	if (input->points.size() < 5)
	{
		return;
	}
	//if (input->points.size() > NumMaxPCLActor)
	//{
	//	input->points.resize(NumMaxPCLActor);
	//}

	//一番最初は空なので、そのままいれる
	if (actorPoints.empty())
	{
		actorPoints.resize(NumMaxPCLActor);
		for (int i = 0; i < actorPoints.size(); i++)
		{
			actorPoints[i].x = -9999;
			actorPoints[i].y = -9999;
			actorPoints[i].z = -9999;
		}
		return;
	}

	//それ以外は、現状のものに近しいものを採用していく
	//現状のものを基準に走査していき、該当しそうなものがない場合は削除
	//kdTreeのRadius Searchで一定範囲内のポイントを検索し、該当するものがあれば採用
	//なければそのインデックスを空ける
	//新規で何か出現した場合は、近傍でないインデックスのやつがそれっぽいので近傍で引っかかったやつをストックしておいてそれ以外を採用
	std::vector<int> usedIndices(0);

	//kdTree
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(input);
	float radius = 0.6f;
	for (int i = 0; i < actorPoints.size(); i++)
	{
		pcl::PointXYZ searchP;
		searchP.x = actorPoints[i].x;
		searchP.y = actorPoints[i].y;
		searchP.z = actorPoints[i].z;

		if (searchP.x < -3600)
		{
			continue;
		}

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		int result = tree.radiusSearch(searchP, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if (result > 0)
		{
			//一番距離が近い奴を採用
			//とりあえず一番距離が近い奴をピックアップ
			int nearestIndex = 0;
			float dist = 100000;
			for (int j = 0; j < pointRadiusSquaredDistance.size(); j++)
			{
				int index = pointIdxRadiusSearch[j];
				//cout << "Point: " << i << ": (" << searchP.x << ", " << searchP.y << ", " << searchP.z << ")" << "---"
				//	<< pointIdxRadiusSearch[j] << ": (" << inputCentroids[index].x() << ", " << inputCentroids[index].y() << ", " << inputCentroids[index].z() << ")" << endl;
				if (pointRadiusSquaredDistance[j] < dist)
				{
					dist = pointRadiusSquaredDistance[j];
					nearestIndex = pointIdxRadiusSearch[j];
				}
			}

			//index = i の点に関しては、nearestIndexの点の情報を採用
			//Low Pass Filter
			//float distX = input->points[nearestIndex].x - actorPoints[i].x;
			//float distY = input->points[nearestIndex].y - actorPoints[i].y;
			//float distZ = input->points[nearestIndex].z - actorPoints[i].z;

			actorPoints[i].x = actorPoints[i].x * (1.0 - MoveRatio) + input->points[nearestIndex].x * MoveRatio;
			actorPoints[i].y = actorPoints[i].y * (1.0 - MoveRatio) + input->points[nearestIndex].y * MoveRatio;
			actorPoints[i].z = actorPoints[i].z * (1.0 - MoveRatio) + input->points[nearestIndex].z * MoveRatio;


			//使ったやつはストック
			usedIndices.push_back(nearestIndex);
		}
		else {
			//そもそも検出されなかった場合は、そのインデックスを不正値で埋めとく
			actorPoints[i].x = -9999;
			actorPoints[i].y = -9999;
			actorPoints[i].z = -9999;
		}
	}

	//新規っぽいポイントたちを採用していく
	for (int i = 0; i < input->points.size(); i++)
	{
		bool used = false;

		for (int j = 0; j < usedIndices.size(); j++)
		{
			if (i == usedIndices[j])
			{
				used = true;
				//break;
			}
		}


		//未使用だったら、空いてそうなインデックスにぶっこむ
		if (!used)
		{
			//cout << "Unused: " << ": (" << inputCentroids[i].x() << ", " << inputCentroids[i].y() << ", " << inputCentroids[i].z() << ")" << endl;
			for (int j = 0; j < actorPoints.size(); j++)
			{
				if (actorPoints[j].z < -1.f)
				{
					actorPoints[j] = input->points[i];
					usedIndices.push_back(i);
					break;
				}
			}
		}
	}

	//点がかぶってる箇所を削除
	pcl::PointCloud<pcl::PointXYZ>::Ptr aP(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < actorPoints.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = actorPoints[i].x;
		p.y = actorPoints[i].y;
		p.z = actorPoints[i].z;
		aP->points.push_back(p);
	}

	//kdTree again
	pcl::search::KdTree<pcl::PointXYZ> tree2;
	tree2.setInputCloud(aP);
	float radius2 = 0.05f;
	for (int i = 0; i < actorPoints.size(); i++)
	{
		pcl::PointXYZ searchP = actorPoints[i];
		if (searchP.x < -3600)
		{
			continue;
		}

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		int result = tree2.radiusSearch(searchP, radius2, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if (result > 0)
		{
			int nearestIndex = 0;
			float dist = 100000;
			for (int j = 0; j < pointRadiusSquaredDistance.size(); j++)
			{
				int index = pointIdxRadiusSearch[j];
				if (index == i)
				{
					continue;
				}
				
				if (pointRadiusSquaredDistance[j] < radius2)
				{
					actorPoints[i].x = -9999;
					actorPoints[i].y = -9999;
					actorPoints[i].z = -9999;
				}
			}
		}
	}
}

void UPCL_BPLibrary::calcCentroid() {
	if (!calcCloud)
	{
		return;
	}
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	mVLPMutex.lock();
	pcl::copyPointCloud(*calcCloud, *cloud);
	mVLPMutex.unlock();
	
	//Z平面にプロジェクション
	pcl::PointCloud<PointType>::Ptr prjCloud = projectionToZ(cloud, 0);
	voxelGridFilter(CentroidVoxelSize, prjCloud);
	euclideanClusterExtraction(prjCloud, eachClouds);
	//重心計算
	std::vector<Eigen::Vector4f> tmpCentroids;
	
	for (int i = 0; i < eachClouds.size(); i++)
	{
		//クラスタの重心を計算
		Eigen::Vector4f center = centroid(eachClouds[i]);
		tmpCentroids.push_back(center);
	}

	calcTrackingCentroids(tmpCentroids);
	//UE_LOG(PCLPlugin, Warning, TEXT("Centroid: %d"), centroids.size());
}

void UPCL_BPLibrary::calcTrackingCentroids(std::vector<Eigen::Vector4f> &inputCentroids) {
	if (inputCentroids.empty())
	{
		return;
	}
	numMaxCentroids = 100;
	if (inputCentroids.size() > numMaxCentroids)
	{
		inputCentroids.resize(numMaxCentroids);
	}

	//一番最初は空なので、そのままいれる
	if (centroids.empty())
	{
		centroids.resize(100);
		for (int i = 0; i < centroids.size(); i++)
		{
			centroids[i].x() = -999;
			centroids[i].y() = -999;
			centroids[i].z() = -999;
		}
		return;
	}

	//それ以外は、現状のものに近しいものを採用していく
	//現状のものを基準に走査していき、該当しそうなものがない場合は削除
	//kdTreeのRadius Searchで一定範囲内のポイントを検索し、該当するものがあれば採用
	//なければそのインデックスを空ける
	//新規で何か出現した場合は、近傍でないインデックスのやつがそれっぽいので近傍で引っかかったやつをストックしておいてそれ以外を採用
	std::vector<int> usedIndices(0);

	//まず重心点たちをポイントクラウドに変換
	pcl::PointCloud<pcl::PointXYZ>::Ptr centroidCloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < inputCentroids.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = inputCentroids[i].x();
		p.y = inputCentroids[i].y();
		p.z = inputCentroids[i].z();

		centroidCloud->points.push_back(p);
	}
	//kdTree
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(centroidCloud);
	float radius = 0.1f;
	for (int i = 0; i < centroids.size(); i++)
	{
		pcl::PointXYZ searchP;
		searchP.x = centroids[i].x();
		searchP.y = centroids[i].y();
		searchP.z = centroids[i].z();
		//
		//if (searchP.x < -100.f)
		//{
		//	continue;
		//}


		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		int result = tree.radiusSearch(searchP, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		if (result > 0)
		{
			//一番距離が近い奴を採用
			//とりあえず一番距離が近い奴をピックアップ
			int nearestIndex = 0;
			float dist = 100000;
			for (int j = 0; j < pointRadiusSquaredDistance.size(); j++)
			{
				int index = pointIdxRadiusSearch[j];
				//cout << "Point: " << i << ": (" << searchP.x << ", " << searchP.y << ", " << searchP.z << ")" << "---"
				//	<< pointIdxRadiusSearch[j] << ": (" << inputCentroids[index].x() << ", " << inputCentroids[index].y() << ", " << inputCentroids[index].z() << ")" << endl;
				if (pointRadiusSquaredDistance[j] < dist)
				{
					dist = pointRadiusSquaredDistance[j];
					nearestIndex = pointIdxRadiusSearch[j];
				}
			}

			//index = i の点に関しては、nearestIndexの点の情報を採用
			centroids[i] = inputCentroids[pointIdxRadiusSearch[0]];

			//使ったやつはストック
			usedIndices.push_back(pointIdxRadiusSearch[0]);
		}
		else {
			//そもそも検出されなかった場合は、そのインデックスを不正値で埋めとく
			centroids[i].x() = -999.f;
			centroids[i].y() = -999.f;
			centroids[i].z() = -999.f;
		}
	}

	//新規っぽいポイントたちを採用していく
	for (int i = 0; i < inputCentroids.size(); i++)
	{
		bool used = false;

		for (int j = 0; j < usedIndices.size(); j++)
		{
			if (i == usedIndices[j])
			{
				used = true;
				//break;
			}
		}


		//未使用だったら、空いてそうなインデックスにぶっこむ
		if (!used)
		{
			//cout << "Unused: " << ": (" << inputCentroids[i].x() << ", " << inputCentroids[i].y() << ", " << inputCentroids[i].z() << ")" << endl;
			for (int j = 0; j < centroids.size(); j++)
			{
				if (centroids[j].x() < -100.f)
				{
					centroids[j] = inputCentroids[i];
					usedIndices.push_back(i);
					break;
				}
			}
		}
	}
}

void UPCL_BPLibrary::euclideanClusterExtraction(pcl::PointCloud<PointType>::Ptr cloud, std::vector<pcl::PointCloud<PointType>::Ptr> &outputCloud) {
	//Clusterに分割
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(Tolerance); // 30cm
	ec.setMinClusterSize(MinRequiredPoint);
	ec.setMaxClusterSize(MaxEnablePoint);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	//分割されたインデックスを元に、クラウドを分割して返す
	//Reset
	for (int i = 0; i < outputCloud.size(); i++)
	{
		outputCloud[i].reset();
	}
	outputCloud.clear();

	//Thread
	boost::thread_group ths;
	for (int i = 0; i < cluster_indices.size(); i++)
	{
		pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
		outputCloud.push_back(p);
		ths.create_thread(boost::bind(&UPCL_BPLibrary::splitCloud, this, cloud, outputCloud[i], cluster_indices[i]));
	}
	ths.join_all();
}

pcl::PointCloud<PointType>::Ptr UPCL_BPLibrary::projectionToZ(pcl::PointCloud<PointType>::Ptr cloud, float zValue) {
	//与えられたZの値にある平面にクラウドをプロジェクションする
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = zValue;

	// Create the filtering object
	pcl::PointCloud<PointType>::Ptr cloud_projected(new pcl::PointCloud<PointType>());
	pcl::ProjectInliers<PointType> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	return cloud_projected;
}

void UPCL_BPLibrary::splitCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr ouputCloud, pcl::PointIndices &indices) {
	for (int j = 0; j < indices.indices.size(); j++)
	{
		int index = indices.indices[j];
		ouputCloud->push_back(inputCloud->points[index]);
	}
}

Eigen::Vector4f UPCL_BPLibrary::centroid(pcl::PointCloud<PointType>::Ptr cloud)
{
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);//重心を計算

	return xyz_centroid;
}


float UPCL_BPLibrary::Clamp(float val, float min, float max) {
	float range = max - min;
	if (range == 0)
	{
		return 0;
	}

	return (val - min) / range;
}