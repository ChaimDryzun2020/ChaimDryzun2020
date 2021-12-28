#include "StdAfx.h"
#include "CamerasManagementClass.h"
#include "RealSenseBase.h"
#include "CameraInterface.h"

CameraManagement::CameraManagement()
{
	m_vCameras.clear();
	m_mRSpipelines.clear();

	m_vCameras.push_back(CreateCamera(0));
}

void CameraManagement::CamerasScan()
{
	RealSenseCamerasScan();
}

CameraInterfacePtr CameraManagement::GetCamera(int Index)
{
	if ((Index < 0) || (Index >= m_vCameras.size()))
	{
		CameraInterfacePtr NullCamera;
		NullCamera = CreateCamera(0);
		return NullCamera;
	}
	return m_vCameras[Index];
}

void CameraManagement::RealSenseCamerasScan()
{
	rs2::context ctx;

	//std::vector <rs2_stream> stream_requests = { RS2_STREAM_DEPTH, RS2_STREAM_COLOR, RS2_STREAM_INFRARED, RS2_STREAM_GYRO , RS2_STREAM_ACCEL };
	std::vector <rs2_stream> stream_requests = { RS2_STREAM_DEPTH, RS2_STREAM_COLOR };
	
	std::vector<std::string> vCamerasIDs;
	for (auto Camera_i : m_vCameras)
	{
		vCamerasIDs.push_back(Camera_i->GetID());
	}

	std::string cameraName, cameraID;
	for (auto&& dev : ctx.query_devices())
	{
		std::vector <rs2_stream> unavailable_streams = stream_requests;
		cameraName = "";
		cameraID = "";

		std::map<rs2_stream, bool> found_streams;
		for (auto& type : stream_requests)
		{
			found_streams[type] = false;
			for (auto& sensor : dev.query_sensors())
			{
				for (auto& profile : sensor.get_stream_profiles())
				{
					if (profile.stream_type() == type)
					{
						found_streams[type] = true;
						unavailable_streams.erase(std::remove(unavailable_streams.begin(), unavailable_streams.end(), type), unavailable_streams.end());
					}
				}
			}
		}

		bool found_all_streams = true;
		for (auto& stream : found_streams)
		{
			if (!stream.second)
			{
				found_all_streams = false;
				break;
			}
		}

		if (found_all_streams)
		{
			if (dev.supports(RS2_CAMERA_INFO_NAME))
				cameraName = dev.get_info(RS2_CAMERA_INFO_NAME);
			if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
				cameraID = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
						
			vCamerasIDs.erase(std::remove(vCamerasIDs.begin(), vCamerasIDs.end(), cameraID), vCamerasIDs.end());
			
			cameraName.erase(remove_if(cameraName.begin(), cameraName.end(), isspace), cameraName.end());
			std::for_each(cameraName.begin(), cameraName.end(), [](char& c) {
				c = ::toupper(c);
				});
			int CameraTypeNum = CameraNameToEnum(cameraName);
			if ((CameraTypeNum > CamerasEnum::NULL_CAMERA) && (CameraTypeNum < CamerasEnum::LAST_CAMERA_INDEX))
			{
				bool existingCamera = false;
				for (auto Camera_i : m_vCameras)
				{
					if (Camera_i->GetID() == cameraID)
					{
						existingCamera = true;
						break;
					}
				}
				if (!existingCamera)
				{
					m_vCameras.push_back(CreateCamera(CameraTypeNum));
					m_vCameras[(int)m_vCameras.size() - 1]->SetID(cameraID);
					m_vCameras[(int)m_vCameras.size() - 1]->SetName(dev.get_info(RS2_CAMERA_INFO_NAME));
					m_vCameras[(int)m_vCameras.size() - 1]->SetType(dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE));
					rs2::pipeline pipe(ctx);
					rs2::config cfg;
					cfg.enable_stream(RS2_STREAM_DEPTH);
					cfg.enable_stream(RS2_STREAM_COLOR);
					cfg.enable_device(cameraID);
					pipe.start(cfg);
					m_mRSpipelines[(int)m_vCameras.size() - 1] = pipe;
					RealSenseBasePtr RSPtr = dynamic_cast<RealSenseBase*>(m_vCameras[m_vCameras.size() - 1].GetPtr());				
					RSPtr->SetPipe(pipe);
				}
			}
		}
	}

	std::vector<int> vIndexCamerasToDelete;
	std::string TmpId;

	for (auto ID_i : vCamerasIDs)
	{
		int Index = 0;
		for (auto Camera_i : m_vCameras)
		{
			TmpId = Camera_i->GetID();
			if (TmpId == ID_i)
			{
				vIndexCamerasToDelete.push_back(Index);
			}
			Index++;
		}
	}
	std::sort(vIndexCamerasToDelete.begin(), vIndexCamerasToDelete.end(), std::greater<int>());

	std::map<int, rs2::pipeline>::iterator it;
	for (int Indx : vIndexCamerasToDelete)
	{
		it = m_mRSpipelines.find(Indx);
		if (it != m_mRSpipelines.end())
		{
			m_mRSpipelines[Indx].stop();
			m_mRSpipelines.erase(Indx);
		}
		m_vCameras.erase(m_vCameras.begin() + Indx);		
	}
}

int CameraManagement::CameraNameToEnum(std::string sCameraName)
{
	if (sCameraName == "INTELREALSENSED455")
		return CamerasEnum::REALSENSE_D455;
	else if ((sCameraName == "INTELREALSENSED435I") || (sCameraName == "INTELREALSENSED435"))
		return CamerasEnum::REALSENSE_D435;
	else if (sCameraName == "INTELREALSENSEL515")
		return CamerasEnum::REALSENSE_L515;
	return CamerasEnum::NULL_CAMERA;
}
