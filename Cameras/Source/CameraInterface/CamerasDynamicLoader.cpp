#include "StdAfx.h"
#include "CamerasDynamicLoader.h"
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

#include <windows.h> 
#include <stdio.h> 

typedef bool(__cdecl* MYPROC)(int, CameraInterface**);
typedef char*(__cdecl* CAMERANAME)();

CameraInterfacePtr CamerasFactory::CreateCamera(int iCameraType)
{
	for (const auto& entry : fs::directory_iterator(m_sPath))
	{
		auto extns = entry.path().extension();
		std::string extn_string{ extns.u8string() };
		std::for_each(extn_string.begin(), extn_string.end(), [](char& c) {
			c = ::toupper(c);
			});
		if (extn_string != ".DLL")
			continue;

		HINSTANCE hinstLib;
		MYPROC ProcAdd;
		BOOL fFreeResult, fRunTimeLinkSuccess = FALSE;

		std::cout << "\t\t" << entry.path().filename() << "\n";
		std::string path_string{ entry.path().filename().u8string() };
		hinstLib = LoadLibraryA("D:\\Dev\\SkillReal\\pointar-research-api\\Bin\\Features\\CamerasDLLs\\REALSENSE_L515.dll");

		if (hinstLib != NULL)
		{
			ProcAdd = (MYPROC)GetProcAddress(hinstLib, "CreateCamera");
			if (NULL != ProcAdd)
			{
				fRunTimeLinkSuccess = TRUE;
				CameraInterfacePtr CameraPtr;
				bool rbReses = (ProcAdd)(iCameraType, (CameraInterface**)CameraPtr.GetPtrPtr());
				if (rbReses)
					return CameraPtr;

			}
			fFreeResult = FreeLibrary(hinstLib);
		}
	}
	return NULL;
}

CamerasFactory::CamerasFactory(void)
{
}

CamerasFactory::~CamerasFactory(void)
{
}

std::vector<std::string> CamerasFactory::GetSupportedCameras()
{
	std::vector<std::string> SupportedCameras;

	for (const auto& entry : fs::directory_iterator(m_sPath))
	{
		auto extns = entry.path().extension();
		std::string extn_string{ extns.u8string() };
		std::for_each(extn_string.begin(), extn_string.end(), [](char& c) {
			c = ::toupper(c);
			});
		if (extn_string != ".DLL")
			continue;

		HINSTANCE hinstLib;
		CAMERANAME ProcAdd;
		BOOL fFreeResult, fRunTimeLinkSuccess = FALSE;

		std::cout << "\t\t" << entry.path().filename() << "\n";
		std::string path_string{ entry.path().filename().u8string() };
		hinstLib = LoadLibraryA("D:\\Dev\\SkillReal\\pointar-research-api\\Bin\\Features\\CamerasDLLs\\REALSENSE_L515.dll");

		if (hinstLib != NULL)
		{
			ProcAdd = (CAMERANAME)GetProcAddress(hinstLib, "GetCameraName");
			if (NULL != ProcAdd)
			{
				fRunTimeLinkSuccess = TRUE;
				std::string CameraName_i;
				CameraName_i = (ProcAdd)();
				if (CameraName_i.size() > 0)
					SupportedCameras.push_back(CameraName_i);
			}
			fFreeResult = FreeLibrary(hinstLib);
		}
	}	

	return SupportedCameras;
}
