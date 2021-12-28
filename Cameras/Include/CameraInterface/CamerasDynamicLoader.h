#pragma once

#include "CameraInterface.h"


class  CamerasFactory
{
public:
	CamerasFactory(void);
	~CamerasFactory(void);

	std::vector<std::string> GetSupportedCameras();
	CameraInterfacePtr CreateCamera(int iCameraType);
	void SetPath(const std::string sPath) { m_sPath = sPath; }

private:
	std::string m_sPath;	
};
