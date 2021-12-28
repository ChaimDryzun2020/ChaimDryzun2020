#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"

typedef struct CameraDimensionsData
{
public:
	int m_iHeight, m_iWidth;
	CameraDimensionsData() : m_iHeight(1080), m_iWidth(1920) { }
	CameraDimensionsData(int iWidth, int iHeight) : m_iHeight(iHeight), m_iWidth(iWidth) { }
	CameraDimensionsData(const CameraDimensionsData &rCopy) : m_iHeight(rCopy.m_iHeight), m_iWidth(rCopy.m_iWidth)	{ }
	~CameraDimensionsData() { }
} CameraDimensionsData;

typedef struct CameraFocalLengthData
{
public:
	float m_fFx, m_fFy;
	CameraFocalLengthData() : m_fFx(750.0f), m_fFy(750.0f) { }
	CameraFocalLengthData(float fFx, float fFy) : m_fFx(fFx), m_fFy(fFy) { }
	CameraFocalLengthData(const CameraFocalLengthData& rCopy) : m_fFx(rCopy.m_fFx), m_fFy(rCopy.m_fFy) { }
	~CameraFocalLengthData() { }

} CameraFocalLengthData;

typedef struct CameraPrinciplePointData
{
public:
	float m_fCx, m_fCy;
	CameraPrinciplePointData() : m_fCx(960.0f), m_fCy(540.0f) { }
	CameraPrinciplePointData(float fCx, float fCy) : m_fCx(fCx), m_fCy(fCy) { }
	CameraPrinciplePointData(const CameraPrinciplePointData& rCopy) : m_fCx(rCopy.m_fCx), m_fCy(rCopy.m_fCy) { }
	~CameraPrinciplePointData() { }

} CameraPrinciplePointData;

typedef struct CameraDistortionCenterData
{
public:
	float m_fCx, m_fCy;
	CameraDistortionCenterData() : m_fCx(960.0f), m_fCy(540.0f) { }
	CameraDistortionCenterData(float fCx, float fCy) : m_fCx(fCx), m_fCy(fCy) { }
	CameraDistortionCenterData(const CameraDistortionCenterData& rCopy) : m_fCx(rCopy.m_fCx), m_fCy(rCopy.m_fCy) { }
	~CameraDistortionCenterData() { }

} CameraDistortionCenterData;

typedef struct CameraRadialDistortionData
{
public:
	std::vector<float> m_vK;

	CameraRadialDistortionData() 
	{
		m_vK.clear();
	}
	CameraRadialDistortionData(const std::vector<float> &vK) : m_vK(vK) { }
	CameraRadialDistortionData(const CameraRadialDistortionData& rCopy) : m_vK(rCopy.m_vK) { }
	~CameraRadialDistortionData() { }

} CameraRadialDistortionData;

typedef struct CameraTangentialDistortionData
{
public:
	std::vector<float> m_vP;
	CameraTangentialDistortionData()
	{
		m_vP.clear();
	}
	CameraTangentialDistortionData(const std::vector<float>& vP) : m_vP(vP) { }
	CameraTangentialDistortionData(const CameraTangentialDistortionData& rCopy) : m_vP(rCopy.m_vP) { }
	~CameraTangentialDistortionData() { }

} CameraTangentialDistortionData;

typedef struct CameraDistorsionData
{
public:
	CameraRadialDistortionData m_vRadialDistortion;
	CameraTangentialDistortionData m_vTangentialDistortion;
	CameraDistortionCenterData m_vDistortionCenter;
	
	CameraDistorsionData()
	{
		m_vRadialDistortion.m_vK.clear();
		m_vTangentialDistortion.m_vP.clear();
		m_vDistortionCenter.m_fCx = 960.0f;
		m_vDistortionCenter.m_fCy = 540.0f;
	}

	CameraDistorsionData(const CameraDistorsionData& rcopy) : m_vDistortionCenter(rcopy.m_vDistortionCenter), m_vRadialDistortion(rcopy.m_vRadialDistortion), m_vTangentialDistortion(rcopy.m_vTangentialDistortion) { }
	~CameraDistorsionData() { }

} CameraDistorsionData;

typedef struct CameraIntrinsicData
{
public:
	CameraDimensionsData m_vImageSize;
	CameraFocalLengthData m_vFocalLength;
	CameraPrinciplePointData m_vPrinciplePoint;
	CameraDistorsionData m_vDistorsion;

	CameraIntrinsicData()
	{
		m_vImageSize.m_iHeight = 1080;
		m_vImageSize.m_iWidth = 1920;
		m_vFocalLength.m_fFx = 750.0f;
		m_vFocalLength.m_fFy = 750.0f;
		m_vPrinciplePoint.m_fCx = 960.0f;
		m_vPrinciplePoint.m_fCy = 540.0f;
		m_vDistorsion.m_vRadialDistortion.m_vK.clear();
		m_vDistorsion.m_vTangentialDistortion.m_vP.clear();
		m_vDistorsion.m_vDistortionCenter.m_fCx = 960.0f;
		m_vDistorsion.m_vDistortionCenter.m_fCy = 540.0f;
	}
		
	CameraIntrinsicData (const CameraIntrinsicData& rcopy) : m_vImageSize(rcopy.m_vImageSize), m_vFocalLength(rcopy.m_vFocalLength), m_vPrinciplePoint(rcopy.m_vPrinciplePoint), m_vDistorsion(rcopy.m_vDistorsion) { }
	~CameraIntrinsicData() {}
} CameraIntrinsicData;

typedef struct CameraExtrinsicData
{
public:
	Eigen::Matrix4f m_mRigidTransform;

	CameraExtrinsicData()
	{
		m_mRigidTransform.setIdentity();
	}

	CameraExtrinsicData(const Eigen::Matrix4f& mRigidTransform) : m_mRigidTransform(mRigidTransform) { }
	CameraExtrinsicData(const Eigen::Vector3f& vTranslation, const Eigen::Matrix3f& mRotation)
	{
		m_mRigidTransform.setIdentity();
		m_mRigidTransform.block<3, 3>(0, 0) = mRotation;
		m_mRigidTransform.block<3, 1>(0, 3) = vTranslation;
		m_mRigidTransform(3, 3) = (double)1.0;
		m_mRigidTransform(3, 0) = (double)0.0;
		m_mRigidTransform(3, 1) = (double)0.0;
		m_mRigidTransform(3, 2) = (double)0.0;
	}

	CameraExtrinsicData(const CameraExtrinsicData& rcopy) : m_mRigidTransform(rcopy.m_mRigidTransform) { }
	~CameraExtrinsicData() { }

} CameraExtrinsicData;

typedef struct RGBATexture
{
public:
	cv::Mat m_mRGBA;
	CameraDimensionsData m_vImageSize;

	RGBATexture()
	{
		m_vImageSize.m_iHeight = 1080;
		m_vImageSize.m_iWidth = 1920;
		m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080 ), CV_8UC4);
	}

	RGBATexture(const cv::Mat& mRGBA) : m_mRGBA(mRGBA) { }
	RGBATexture(const RGBATexture& rcopy) : m_mRGBA(rcopy.m_mRGBA) , m_vImageSize(rcopy.m_vImageSize) { }
	~RGBATexture() { }
} RGBATexture;

typedef struct DepthTexture
{
public:
	cv::Mat m_mDepth;
	CameraDimensionsData m_vImageSize;

	DepthTexture()
	{
		m_vImageSize.m_iHeight = 1;
		m_vImageSize.m_iWidth = 1;
		m_mDepth = cv::Mat::zeros(1, 1, CV_16FC1);
	}

	DepthTexture(const cv::Mat& mDepth) : m_mDepth(mDepth) { }
	DepthTexture(const DepthTexture& rcopy) : m_mDepth(rcopy.m_mDepth), m_vImageSize(rcopy.m_vImageSize) { }
	~DepthTexture() { }
} DepthTexture;

typedef struct PointData
{
public:
	float m_fX, m_fY, m_fZ;
	PointData() : m_fX(0.0f), m_fY(0.0f), m_fZ(0.0f) { }
	PointData(float fX, float fY, float fZ) : m_fX(fX), m_fY(fY), m_fZ(fZ) { }
	PointData(const PointData& rCopy) : m_fX(rCopy.m_fX), m_fY(rCopy.m_fY), m_fZ(rCopy.m_fZ) { }
	~PointData() { }
} PointData;

typedef struct ColorData
{
public:
	int m_iR, m_iG, m_iB;
	ColorData() : m_iR(0), m_iG(0), m_iB(0) { }
	ColorData(int iR, int iG, int iB) : m_iR(iR), m_iG(iG), m_iB(iB) { }
	ColorData(const ColorData& rCopy) : m_iR(rCopy.m_iR), m_iG(rCopy.m_iG), m_iB(rCopy.m_iB) { }
	~ColorData() { }
} ColorData;

typedef struct PointColorData
{
public:
	PointData m_vPoint;
	ColorData m_vRGB;

	PointColorData()
	{
		m_vPoint.m_fX = 0.0f;
		m_vPoint.m_fY = 0.0f;
		m_vPoint.m_fZ = 0.0f;
		m_vRGB.m_iR = 0;
		m_vRGB.m_iG = 0;
		m_vRGB.m_iB = 0;
	}
	PointColorData(const PointColorData& rcopy) : m_vPoint(rcopy.m_vPoint), m_vRGB(rcopy.m_vRGB) { }
	~PointColorData() { }

} PointColorData;

typedef struct PointCloudData
{
public:
	std::vector<PointColorData> m_vPoints;
	PointCloudData()
	{
		m_vPoints.clear();
	}
	PointCloudData(const PointCloudData& rCopy) : m_vPoints(rCopy.m_vPoints) { }
	~PointCloudData() { }

} PointCloudData;

