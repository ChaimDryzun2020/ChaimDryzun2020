#include "pch.h"
#include "CameraInterfaceAPI.h"


extern "C" SKILLREAL_API const char* DLL_FullVersion()
{
	return CAMERAINTERFACEDLL_VERSION.c_str();
}

extern "C" SKILLREAL_API const char* DLL_MajorVersion()
{
	std::string MAJOR_DLL_VERSION = CAMERAINTERFACEDLL_VERSION.substr(0, CAMERAINTERFACEDLL_VERSION.find("."));
	const char* OutChar = MAJOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API const char* DLL_MinorVersion()
{
	std::string MINOR_DLL_VERSION = CAMERAINTERFACEDLL_VERSION.substr(CAMERAINTERFACEDLL_VERSION.find(".") + 1, CAMERAINTERFACEDLL_VERSION.size());
	const char* OutChar = MINOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API bool CreateCamera(int iCameraType, CameraInterface * *outCameraPtr)
{
	CameraInterfacePtr CameraPtr = NULL;
	CameraPtr = CreateCamera(iCameraType);
	(*outCameraPtr) = CameraPtr;
	return ((*outCameraPtr) != NULL);	
}
