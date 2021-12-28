#include "pch.h"
#include "CamerasManagementAPI.h"

CameraManagement m_CameraManager;

extern "C" SKILLREAL_API const char* DLL_FullVersion()
{
	return CAMERASMANAGEMENTDLL_VERSION.c_str();
}

extern "C" SKILLREAL_API const char* DLL_MajorVersion()
{
	std::string MAJOR_DLL_VERSION = CAMERASMANAGEMENTDLL_VERSION.substr(0, CAMERASMANAGEMENTDLL_VERSION.find("."));
	const char* OutChar = MAJOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API const char* DLL_MinorVersion()
{
	std::string MINOR_DLL_VERSION = CAMERASMANAGEMENTDLL_VERSION.substr(CAMERASMANAGEMENTDLL_VERSION.find(".") + 1, CAMERASMANAGEMENTDLL_VERSION.size());
	const char* OutChar = MINOR_DLL_VERSION.c_str();
	return OutChar;
}

extern "C" SKILLREAL_API int CamerasNum()
{
	return m_CameraManager.GetCamerasNum();
}

//extern "C" SKILLREAL_API CameraInterfacePtr GetCamera(int Index)
//{
//	return m_CameraManager.GetCamera(Index);
//}
//
//extern "C" SKILLREAL_API std::vector<CameraInterfacePtr> GetCameras()
//{
//	return m_CameraManager.GetCameras();
//}
//
