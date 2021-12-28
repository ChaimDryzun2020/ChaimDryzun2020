#include "StdAfx.h"
#include "RealSenseD455.h"

CameraInterfacePtr CreateCameraRealSenseD455()
{
	CameraInterfacePtr p = new IBaseImpl<RealSense_D455>;
	_ASSERTE(p);
	return p;
}
