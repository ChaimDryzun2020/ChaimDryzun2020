#pragma once

#include <iostream>
#include <string>
#include <map>
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <vector>

#include "CameraInterface.h"

class CameraManagement
{
public:
	CameraManagement();
	~CameraManagement() { }
	
	void CamerasScan();
	std::vector<CameraInterfacePtr> GetCameras() { return m_vCameras; }
	CameraInterfacePtr GetCamera(int Index);
	int GetCamerasNum() { return (int)m_vCameras.size(); }

private:
	std::vector<CameraInterfacePtr> m_vCameras;
	
	
	std::map<int, rs2::pipeline> m_mRSpipelines;
		
	void RealSenseCamerasScan();
	int CameraNameToEnum(std::string sCameraName);

};
