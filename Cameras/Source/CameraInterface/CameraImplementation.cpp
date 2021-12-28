#include "StdAfx.h"
#include "CameraBaseImplementation.h"

CameraBaseImplementation::CameraBaseImplementation()
{
	m_sID = "007";
	m_sName = "NULL";
	m_sType = "NULL";
}

bool CameraBaseImplementation::SetRGBATexture(RGBATexture& mRGBATexture, bool bUndistort /*= true*/)
{	
	mRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
	mRGBATexture.m_vImageSize.m_iWidth = 1920;
	mRGBATexture.m_vImageSize.m_iHeight = 1080;
	return true;
}

RGBATexture CameraBaseImplementation::UpdateRGBAImage(bool bUndistort /*= true*/)
{
	RGBATexture TmpRGBATexture;
	TmpRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
	TmpRGBATexture.m_vImageSize.m_iWidth = 1920;
	TmpRGBATexture.m_vImageSize.m_iHeight = 1080;
	return TmpRGBATexture;
}

CameraIntrinsicData CameraBaseImplementation::GetCameraIntrinsic()
{
	CameraIntrinsicData TmpCameraIntrinsicData;
	return TmpCameraIntrinsicData;
}

DepthTexture CameraBaseImplementation::GetDepthMap(bool bAutoStop /*= true*/)
{
	DepthTexture TmpDepthTexture;
	TmpDepthTexture.m_mDepth = cv::Mat::zeros(cv::Size(1920, 1080), CV_16FC1);
	TmpDepthTexture.m_vImageSize.m_iWidth = 1920;
	TmpDepthTexture.m_vImageSize.m_iHeight = 1080;
	return TmpDepthTexture;
}

RGBATexture CameraBaseImplementation::GetHeatMap(bool bAutoStop /*= true*/)
{
	RGBATexture TmpRGBATexture;
	TmpRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
	TmpRGBATexture.m_vImageSize.m_iWidth = 1920;
	TmpRGBATexture.m_vImageSize.m_iHeight = 1080;
	return TmpRGBATexture;
}

std::vector<float> CameraBaseImplementation::GetPointsCloud(bool bAutoStop /*= true*/)
{
	std::vector<float> TmpPointCloud;
	TmpPointCloud.clear();
	return TmpPointCloud;
}

extern CameraInterfacePtr CreateCameraRealSenseL515();
extern CameraInterfacePtr CreateCameraRealSenseD435();
extern CameraInterfacePtr CreateCameraRealSenseD455();

CameraInterfacePtr CreateCamera(int iCameraType)
{
	if ((iCameraType <= CamerasEnum::NULL_CAMERA) || (iCameraType >= CamerasEnum::LAST_CAMERA_INDEX))
	{
		CameraInterface* CameraPtr = new IBaseImpl<CameraBaseImplementation>;
		CameraPtr->AddRef();
		return CameraPtr;
	}

	CameraInterfacePtr CameraPtr = NULL;

	switch (iCameraType) 
	{
	case CamerasEnum::REALSENSE_L515:	CameraPtr = CreateCameraRealSenseL515(); break;
	case CamerasEnum::REALSENSE_D435:	CameraPtr = CreateCameraRealSenseD435(); break;
	case CamerasEnum::REALSENSE_D455:	CameraPtr = CreateCameraRealSenseD455(); break;
	}

	return CameraPtr;
}