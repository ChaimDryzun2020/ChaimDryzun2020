#include "StdAfx.h"
#include "CamerasHelperFunctions.h"

#include <iostream>
#include <string>
#include <map>
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <omp.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

cv::Mat CameraHelperFunctions::RS_frame_to_mat(const rs2::frame& f)
{
	using namespace cv;
	using namespace rs2;

	auto vf = f.as<video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	if (f.get_profile().format() == RS2_FORMAT_BGR8)
	{
		return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_RGB8)
	{
		auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
		Mat r_bgr;
		cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
		return r_bgr;
	}
	else if (f.get_profile().format() == RS2_FORMAT_Z16)
	{
		return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y8)
	{
		return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
	{
		return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}

	throw std::runtime_error("Frame format is not supported yet!");
}

RGBATexture CameraHelperFunctions::CreateHeatMap(const DepthTexture& DepthMap)
{
	cv::Mat mask = DepthMap.m_mDepth > 0;
	double minc[1], maxc[1];
	cv::minMaxLoc(DepthMap.m_mDepth, minc, maxc, NULL, NULL, mask);

	int iHeight = DepthMap.m_mDepth.size().height;
	int iWidth = DepthMap.m_mDepth.size().width;

	RGBATexture OutHeatMap;
	OutHeatMap.m_mRGBA = cv::Mat::zeros(cv::Size(iWidth, iHeight), CV_8UC4);
	OutHeatMap.m_vImageSize.m_iHeight = iHeight;
	OutHeatMap.m_vImageSize.m_iWidth = iWidth;

	std::vector<int> Color_ij;
	float Depth_ij;

	for (int i = 0; i < DepthMap.m_mDepth.cols; i++)
		for (int j = 0; j < DepthMap.m_mDepth.rows; j++)
		{
			Depth_ij = DepthMap.m_mDepth.at<float>(cv::Point(i, j));
			if (Depth_ij > 0.0)
			{
				Color_ij = SetColor(Depth_ij, (float)minc[0], (float)maxc[0]);
				//OutHeatMap.m_mRGBA.at<cv::Vec4b>(i, j) = cv::Vec4b(Color_ij[2], Color_ij[1], Color_ij[0], Color_ij[3]);
				OutHeatMap.m_mRGBA.at<cv::Vec4b>(j, i) = cv::Vec4b(Color_ij[2], Color_ij[1], Color_ij[0], Color_ij[3]);
			}
			else
			{
				//OutHeatMap.m_mRGBA.at<cv::Vec4b>(i, j) = cv::Vec4b(0.0, 0.0, 0.0, 0.0);
				OutHeatMap.m_mRGBA.at<cv::Vec4b>(j, i) = cv::Vec4b((uchar)0.0f, (uchar)0.0f, (uchar)0.0f, (uchar)0.0f);
			}
		}

	return OutHeatMap;
}

std::vector<int> CameraHelperFunctions::SetColor(float Distance, float MinDist, float MaxDist)
{
	std::vector<int> OutColor;
	OutColor.resize(4);

	float AbsDist = fabs(Distance);

	if (AbsDist <= MinDist)
	{
		OutColor[0] = 0;
		OutColor[1] = 0;
		OutColor[2] = 0;
		OutColor[3] = 0;
	}
	else if (AbsDist >= MaxDist)
	{
		OutColor[0] = 0;
		OutColor[1] = 0;
		OutColor[2] = 0;
		OutColor[3] = 0;
	}
	else
	{
		float x = ((AbsDist - MinDist) / (MaxDist - MinDist));
		int InitH = (int)(std::round(x * 240.0f));
		int InversH = 240 - InitH;

		float H;
		H = ((240.0f - (240.0f * x)) / 60.0f);

		if (H < 0.0)
			H = 0.0f;
		if (H > 4.0)
			H = 4.0f;

		float xTmp = H;
		while (xTmp >= 2.0f)
			xTmp -= 2.0;

		int X = (int)std::round(255.0f * (1.0f - fabs(xTmp - 1.0f)));
		int C = 255;
		if (H < 1)
		{
			OutColor[0] = C;
			OutColor[1] = X;
			OutColor[2] = 0;
			OutColor[3] = 255;
		}
		else if (H < 2)
		{
			OutColor[0] = X;
			OutColor[1] = C;
			OutColor[2] = 0;
			OutColor[3] = 255;
		}
		else if (H < 3)
		{
			OutColor[0] = 0;
			OutColor[1] = C;
			OutColor[2] = X;
			OutColor[3] = 255;
		}
		else if (H < 4)
		{
			OutColor[0] = 0;
			OutColor[1] = X;
			OutColor[2] = C;
			OutColor[3] = 255;
		}
		else if (H < 5)
		{
			OutColor[0] = X;
			OutColor[1] = 0;
			OutColor[2] = C;
			OutColor[3] = 255;
		}
		else if (H < 6)
		{
			OutColor[0] = C;
			OutColor[1] = 0;
			OutColor[2] = X;
			OutColor[3] = 255;
		}
	}
	return OutColor;
}

bool CameraHelperFunctions::device_with_streams(std::vector <rs2_stream> stream_requests, std::string& out_serial)
{
	rs2::context ctx;
	auto devs = ctx.query_devices();
	std::vector <rs2_stream> unavailable_streams = stream_requests;
	for (auto dev : devs)
	{
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
						if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
							out_serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
					}
				}
			}
		}
		// Check if all streams are found in current device
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
			return true;
	}
	// After scanning all devices, not all requested streams were found
	for (auto& type : unavailable_streams)
	{
		switch (type)
		{
		case RS2_STREAM_POSE:
		case RS2_STREAM_FISHEYE:
			std::cerr << "Connect T26X and rerun the demo" << std::endl;
			break;
		case RS2_STREAM_DEPTH:
			std::cerr << "The demo requires Realsense camera with DEPTH sensor" << std::endl;
			break;
		case RS2_STREAM_COLOR:
			std::cerr << "The demo requires Realsense camera with RGB sensor" << std::endl;
			break;
		default:
			throw std::runtime_error("The requested stream: " + std::to_string(type) + ", for the demo is not supported by connected devices!"); // stream type
		}
	}
	return false;
}

cv::Mat CameraHelperFunctions::Undistord_CPU(const cv::Mat& InputImage, const CameraIntrinsicData& InputCameraIntrinsicData)
{
	cv::Mat OutputImage;
	
	cv::Mat cam_intrinsic(3, 3, CV_64FC1);
	cam_intrinsic.at<double>(0, 0) = InputCameraIntrinsicData.m_vFocalLength.m_fFx;
	cam_intrinsic.at<double>(1, 0) = 0.0;
	cam_intrinsic.at<double>(2, 0) = 0.0;

	cam_intrinsic.at<double>(0, 1) = 0.0;
	cam_intrinsic.at<double>(1, 1) = InputCameraIntrinsicData.m_vFocalLength.m_fFy;
	cam_intrinsic.at<double>(2, 1) = 0.0;

	cam_intrinsic.at<double>(0, 2) = InputCameraIntrinsicData.m_vPrinciplePoint.m_fCx;
	cam_intrinsic.at<double>(1, 2) = InputCameraIntrinsicData.m_vPrinciplePoint.m_fCy;
	cam_intrinsic.at<double>(2, 2) = 1.0;

	int RadialDistortionSize = (int)InputCameraIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.size();
	int TangentialDistortionSize = (int)InputCameraIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP.size();
	cv::Mat distCoeffs(5, 1, CV_64FC1);
	distCoeffs.at<double>(0) = (RadialDistortionSize > 0) ? (double)InputCameraIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK[0] : 0.0;
	distCoeffs.at<double>(1) = (RadialDistortionSize > 1) ? (double)InputCameraIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK[1] : 0.0;
	distCoeffs.at<double>(2) = (TangentialDistortionSize > 0) ? (double)InputCameraIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP[0] : 0.0;
	distCoeffs.at<double>(3) = (TangentialDistortionSize > 1) ? (double)InputCameraIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP[1] : 0.0;
	distCoeffs.at<double>(4) = (RadialDistortionSize > 2) ? (double)InputCameraIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK[2] : 0.0;

	cv::undistort(InputImage, OutputImage, cam_intrinsic, distCoeffs);

	return OutputImage;
}

PointCloudData CameraHelperFunctions::Deproject_CPU(const DepthTexture& DepthMap, const CameraIntrinsicData& InputCameraIntrinsicData)
{
	std::vector<cv::Point> locations;   
	cv::findNonZero(DepthMap.m_mDepth, locations);
	PointCloudData m_vPointCloud;
	for (auto Point_i : locations)
	{
		PointColorData Pi;
		Pi.m_vPoint.m_fZ = DepthMap.m_mDepth.at<float>(Point_i);
		Pi.m_vPoint.m_fX = ((Point_i.x - InputCameraIntrinsicData.m_vPrinciplePoint.m_fCx) * Pi.m_vPoint.m_fZ) / InputCameraIntrinsicData.m_vFocalLength.m_fFx;
		Pi.m_vPoint.m_fY = ((Point_i.y - InputCameraIntrinsicData.m_vPrinciplePoint.m_fCy) * Pi.m_vPoint.m_fZ) / InputCameraIntrinsicData.m_vFocalLength.m_fFy;
		Pi.m_vRGB.m_iR = 0;
		Pi.m_vRGB.m_iG = 0;
		Pi.m_vRGB.m_iB = 0;
		m_vPointCloud.m_vPoints.push_back(Pi);
	}
	return m_vPointCloud;
}

PointCloudData CameraHelperFunctions::Deproject_CPU(const DepthTexture& DepthMap, const RGBATexture& ColorImage, const CameraIntrinsicData& InputCameraIntrinsicData)
{
	std::vector<cv::Point> locations;
	cv::findNonZero(DepthMap.m_mDepth, locations);
	PointCloudData m_vPointCloud;
	for (auto Point_i : locations)
	{
		PointColorData Pi;
		Pi.m_vPoint.m_fZ = DepthMap.m_mDepth.at<float>(Point_i);
		Pi.m_vPoint.m_fX = ((Point_i.x - InputCameraIntrinsicData.m_vPrinciplePoint.m_fCx) * Pi.m_vPoint.m_fZ) / InputCameraIntrinsicData.m_vFocalLength.m_fFx;
		Pi.m_vPoint.m_fY = ((Point_i.y - InputCameraIntrinsicData.m_vPrinciplePoint.m_fCy) * Pi.m_vPoint.m_fZ) / InputCameraIntrinsicData.m_vFocalLength.m_fFy;
		cv::Vec4b bgraPixel = ColorImage.m_mRGBA.at<cv::Vec4b>(Point_i.y, Point_i.x);
		Pi.m_vRGB.m_iR = bgraPixel[0];
		Pi.m_vRGB.m_iG = bgraPixel[1];
		Pi.m_vRGB.m_iB = bgraPixel[2];
		m_vPointCloud.m_vPoints.push_back(Pi);
	}
	return m_vPointCloud;
}
