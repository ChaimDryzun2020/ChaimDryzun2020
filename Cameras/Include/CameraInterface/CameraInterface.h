// CameraInterface.h
// This is the Header file for the basic abstract camera interface
// This will be the structure that the app will know, 
// and all the functionality will be defined here
//
//	Important: PLEASE ADD NEW FUNCTIONALITIES AT THE END OF THE STRUCTURE
//
// First created by Chaim Dryzun at 11.11.2021

#pragma once

#include "BaseImplementation.h"
#include "CameraDataStructures.h"
#include "CamerasEnum.h"
#include <string>
#include <vector>

// Declaring the structure and the smart pointer
struct CameraInterface;
typedef CSmartPtr<CameraInterface> CameraInterfacePtr;

// Defining the general camera abstract interface structure
struct CameraInterface : public virtual IBase
{
	virtual ~CameraInterface() {}	// Destructor

	// Returning the camera type, bases on the enum in CamerasEnum.h
	virtual int GetCameraType() = 0;

	// Getting the Camera's name, ID and type
	virtual std::string GetID() = 0;
	virtual std::string GetName() = 0;
	virtual std::string GetType() = 0;

	// Setting the Camera's name, ID and type
	virtual void SetID(const std::string& sID) = 0;
	virtual void SetName(const std::string& sName) = 0;
	virtual void SetType(const std::string& sType) = 0;

	// If the image needs to be rotated
	virtual void Rotate180(bool bRotate = false) = 0;

	// A boolean function which tells if the camera supports RGB images
	virtual bool SupportsRGB() = 0;

	virtual bool SetRGBATexture(RGBATexture & mRGBATexture, bool bUndistort = true) = 0;
	virtual bool RGBStream(bool bStreamRGB) = 0;
	virtual RGBATexture UpdateRGBAImage(bool bUndistort = true) = 0;

	// Get the intrinsic cameras parameters of the relevant RGB camera
	virtual CameraIntrinsicData GetCameraIntrinsic() = 0;

	// Get depth map
	virtual DepthTexture GetDepthMap(bool bAutoStop = true) = 0;

	// Get the depth map as relative colored heat map
	virtual RGBATexture GetHeatMap(bool bAutoStop = true) = 0;

	// Get point cloud - a set of 3D points
	virtual std::vector<float> GetPointsCloud(bool bAutoStop = true) = 0;
};

extern CameraInterfacePtr CreateCamera(int iCameraType);