#include "StdAfx.h"
#include "RealSenseL515.h"

CameraInterfacePtr CreateCameraRealSenseL515()
{
	CameraInterfacePtr p = new IBaseImpl<RealSense_L515>;
	_ASSERTE(p);
	return p;
}

//
//extern "C"
//{
//	__declspec(dllexport) bool CreateCamera(int iCameraType, CameraInterface** outCameraPtr)
//	{
//		if (iCameraType != CamerasEnum::REALSENSE_L515) {
//			(*outCameraPtr) = NULL;
//			return false;
//		}
//
//		CameraInterface* CameraPtr = new IBaseImpl<RealSense_L515>;
//		CameraPtr->AddRef();
//
//		(*outCameraPtr) = CameraPtr;
//		return ((*outCameraPtr) != NULL);
//	}
//};
//
//
//extern "C"
//{
//	__declspec(dllexport) const char* GetCameraName(int iCameraType)
//	{
//		if (iCameraType == CamerasEnum::REALSENSE_L515)
//			return "RealSenseL515";
//		return "";
//	}
//};