#pragma once

#include "RealSenseBase.h"

class RealSense_L515 : public RealSenseBase
{
public:
	RealSense_L515() : RealSenseBase()
	{
		m_sID = "XXXXXXXX";
		m_sName = "RealSense_L515";
		m_sType = "RealSense_L515";

	}
	virtual ~RealSense_L515() { }

	virtual int GetCameraType() { return CamerasEnum::REALSENSE_L515; }
};

typedef CSmartPtr<RealSense_L515> RealSenseL515Ptr;