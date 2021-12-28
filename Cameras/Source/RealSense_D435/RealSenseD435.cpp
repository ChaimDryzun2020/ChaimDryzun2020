#include "StdAfx.h"
#include "RealSenseD435.h"

CameraInterfacePtr CreateCameraRealSenseD435()
{
	CameraInterfacePtr p = new IBaseImpl<RealSense_D435>;
	_ASSERTE(p);
	return p;
}
