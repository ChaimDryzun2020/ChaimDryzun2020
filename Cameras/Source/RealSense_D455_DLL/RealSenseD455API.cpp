#include "pch.h"
#include "RealSenseD455API.h"

extern "C" SKILLREAL_API const char* DLL_FullVersion()
{
	return REALSENSED455DLL_VERSION.c_str();
}

extern "C" SKILLREAL_API const char* DLL_MajorVersion()
{
	std::string MAJOR_DLL_VERSION = REALSENSED455DLL_VERSION.substr(0, REALSENSED455DLL_VERSION.find("."));
	const char* OutChar = MAJOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API const char* DLL_MinorVersion()
{
	std::string MINOR_DLL_VERSION = REALSENSED455DLL_VERSION.substr(REALSENSED455DLL_VERSION.find(".") + 1, REALSENSED455DLL_VERSION.size());
	const char* OutChar = MINOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API bool CreateCamera(int iCameraType, CameraInterface * *outCameraPtr)
{
	if (iCameraType != CamerasEnum::REALSENSE_D455) {
		(*outCameraPtr) = NULL;
		return false;
	}

	CameraInterface* CameraPtr = new IBaseImpl<RealSense_D455>;
	CameraPtr->AddRef();

	(*outCameraPtr) = CameraPtr;
	return ((*outCameraPtr) != NULL);
}

extern "C" SKILLREAL_API const char* GetCameraName()
{
	return "RealSenseD455";
}

