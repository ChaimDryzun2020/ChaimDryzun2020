#pragma once

#include "RealSenseBase.h"

class RealSense_D455 : public RealSenseBase
{
public:
	RealSense_D455() : RealSenseBase()
	{
		m_sID = "XXXXXXXX";
		m_sName = "RealSense_L455";
		m_sType = "RealSense_L455";

	}
	virtual ~RealSense_D455() { }

	virtual int GetCameraType() { return CamerasEnum::REALSENSE_D455; }
};

typedef CSmartPtr<RealSense_D455> RealSenseD455Ptr;
