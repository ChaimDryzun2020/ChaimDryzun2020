#pragma once

#include <librealsense2/rs.hpp>
#include "CameraDataStructures.h"

namespace CameraHelperFunctions
{
	cv::Mat RS_frame_to_mat(const rs2::frame& f);
	RGBATexture CreateHeatMap(const DepthTexture& DepthMap);
	std::vector<int> SetColor(float Distance, float MinDist, float MaxDist);
	bool device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial);
	cv::Mat Undistord_CPU(const cv::Mat& InputImage, const CameraIntrinsicData& InputCameraIntrinsicData);
	PointCloudData Deproject_CPU(const DepthTexture& DepthMap, const CameraIntrinsicData& InputCameraIntrinsicData);
	PointCloudData Deproject_CPU(const DepthTexture& DepthMap, const RGBATexture& ColorImage, const CameraIntrinsicData& InputCameraIntrinsicData);
}
