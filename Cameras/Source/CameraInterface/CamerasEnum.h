#pragma once

struct CamerasEnum
{
	enum CameraType
	{
		REALSENSE_BASE = -3,
		Template = -2,
		UNKNOWN = -1,
		NULL_CAMERA = 0,
		IMPORT = 1,
		REALSENSE_L515 = 2,
		REALSENSE_D435 = 3,
		REALSENSE_D455 = 4,
		ZIVID_ONE_PLUS_LARGE = 5,
		ZIVID_TWO = 6,
		MICROSOFT_KINECT = 7,
		LAST_CAMERA_INDEX
	};
};
