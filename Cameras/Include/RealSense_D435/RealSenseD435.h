#pragma once

#include "RealSenseBase.h"

class RealSense_D435 : public RealSenseBase
{
public:
	RealSense_D435() : RealSenseBase()
	{
		m_sID = "XXXXXXXX";
		m_sName = "RealSense_L435";
		m_sType = "RealSense_L435";

	}
	virtual ~RealSense_D435() { }

	virtual int GetCameraType() { return CamerasEnum::REALSENSE_D435; }
};

typedef CSmartPtr<RealSense_D435> RealSenseD435Ptr;