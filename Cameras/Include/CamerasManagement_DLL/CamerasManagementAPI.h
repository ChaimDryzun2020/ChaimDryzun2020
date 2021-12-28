// CameraManagmentAPI.h - Contains declarations of the API functions
// This is the Header file for functions exposed outside
//
// First created by Chaim Dryzun at 21.12.2021

#pragma once

#include <string>
#include "CamerasManagementClass.h"

#ifdef CAMERASMANAGEMENTDLL_EXPORTS

#define SKILLREAL_API __declspec(dllexport)
#else
#define SKILLREAL_API __declspec(dllimport)
#endif

std::string CAMERASMANAGEMENTDLL_VERSION = "1.00";	// 26.12.2021

extern "C" SKILLREAL_API const char* DLL_FullVersion();
extern "C" SKILLREAL_API const char* DLL_MajorVersion();
extern "C" SKILLREAL_API const char* DLL_MinorVersion();
extern "C" SKILLREAL_API int CamerasNum();
//extern "C" SKILLREAL_API CameraInterfacePtr GetCamera(int Index);
//extern "C" SKILLREAL_API std::vector<CameraInterfacePtr> GetCameras();
