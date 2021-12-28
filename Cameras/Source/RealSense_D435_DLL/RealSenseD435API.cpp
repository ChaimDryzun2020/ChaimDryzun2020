#include "pch.h"
#include "RealSenseD435API.h"

extern "C" SKILLREAL_API const char* DLL_FullVersion()
{
	return REALSENSED435DLL_VERSION.c_str();
}

extern "C" SKILLREAL_API const char* DLL_MajorVersion()
{
	std::string MAJOR_DLL_VERSION = REALSENSED435DLL_VERSION.substr(0, REALSENSED435DLL_VERSION.find("."));
	const char* OutChar = MAJOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API const char* DLL_MinorVersion()
{
	std::string MINOR_DLL_VERSION = REALSENSED435DLL_VERSION.substr(REALSENSED435DLL_VERSION.find(".") + 1, REALSENSED435DLL_VERSION.size());
	const char* OutChar = MINOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API bool CreateCamera(int iCameraType, CameraInterface * *outCameraPtr)
{
	if (iCameraType != CamerasEnum::REALSENSE_D435) {
		(*outCameraPtr) = NULL;
		return false;
	}

	CameraInterface* CameraPtr = new IBaseImpl<RealSense_D435>;
	CameraPtr->AddRef();

	(*outCameraPtr) = CameraPtr;
	return ((*outCameraPtr) != NULL);
}

extern "C" SKILLREAL_API const char* GetCameraName()
{
	return "RealSenseD435";
}

