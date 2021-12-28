#include "pch.h"
#include "RealSenseL515API.h"

extern "C" SKILLREAL_API const char* DLL_FullVersion()
{
	return REALSENSEL515DLL_VERSION.c_str();
}

extern "C" SKILLREAL_API const char* DLL_MajorVersion()
{
	std::string MAJOR_DLL_VERSION = REALSENSEL515DLL_VERSION.substr(0, REALSENSEL515DLL_VERSION.find("."));
	const char* OutChar = MAJOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API const char* DLL_MinorVersion()
{
	std::string MINOR_DLL_VERSION = REALSENSEL515DLL_VERSION.substr(REALSENSEL515DLL_VERSION.find(".") + 1, REALSENSEL515DLL_VERSION.size());
	const char* OutChar = MINOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API bool CreateCamera(int iCameraType, CameraInterface * *outCameraPtr)
{
	if (iCameraType != CamerasEnum::REALSENSE_L515) {
		(*outCameraPtr) = NULL;
		return false;
	}

	CameraInterface* CameraPtr = new IBaseImpl<RealSense_L515>;
	CameraPtr->AddRef();

	(*outCameraPtr) = CameraPtr;
	return ((*outCameraPtr) != NULL);
}

extern "C" SKILLREAL_API const char* GetCameraName()
{
	return "RealSenseL515";
}

