#include "StdAfx.h"
#include "RealSenseBase.h"
#include "CamerasHelperFunctions.h"

RealSenseBase::RealSenseBase()
{
	m_bAllowRS = true; 
	m_bRotate = false;
	m_bStreamRGB = false;
	m_sID = "XXXXXXXX";
	m_sName = "REALSENSE_BASE";
	m_sType = "REALSENSE_BASE";
	m_bColorIntrinsicData = false;	
	m_bIrIntrinsicData = false;

}

void RealSenseBase::SetPipe(rs2::pipeline& rPipe)
{
	m_rPipe = rPipe;
}

bool RealSenseBase::SetRGBATexture(RGBATexture& mRGBATexture, bool bUndistort /*= true*/)
{
	while (m_bStreamRGB)
	{
		if (m_bAllowRS)
		{
			rs2::align align_to_color(RS2_STREAM_COLOR);
			auto frames = m_rPipe.wait_for_frames();
			frames.apply_filter(align_to_color);
			frames = align_to_color.process(frames);

			if (auto color_frame = frames.get_color_frame())
			{
				cv::Mat InitialImage;
				InitialImage = CameraHelperFunctions::RS_frame_to_mat(color_frame);
				cv::Mat RGBA;
				cv::cvtColor(InitialImage, RGBA, cv::COLOR_RGB2BGRA);
				cv::Mat UndistortRGBA;
				if (bUndistort)
					UndistortRGBA = CameraHelperFunctions::Undistord_CPU(RGBA, m_sColorIntrinsicData);
				else
					RGBA.copyTo(UndistortRGBA);

				if (m_bRotate)
					cv::flip(UndistortRGBA, UndistortRGBA, -1);
								
				UndistortRGBA.copyTo(mRGBATexture.m_mRGBA);
				mRGBATexture.m_vImageSize.m_iHeight = UndistortRGBA.size().height;
				mRGBATexture.m_vImageSize.m_iWidth = UndistortRGBA.size().width;
			}
		}
		else
		{
			mRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
			mRGBATexture.m_vImageSize.m_iHeight = 1080;
			mRGBATexture.m_vImageSize.m_iWidth = 1920;
		}
	}
	return true;
}

RGBATexture RealSenseBase::UpdateRGBAImage(bool bUndistort /*= true*/)
{
	m_bStreamRGB = false;

	RGBATexture TmpRGBATexture;
	TmpRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
	TmpRGBATexture.m_vImageSize.m_iHeight = 1080;
	TmpRGBATexture.m_vImageSize.m_iWidth = 1920;

	if (m_bAllowRS)
	{
		rs2::align align_to_color(RS2_STREAM_COLOR);
		auto frames = m_rPipe.wait_for_frames();
		frames = align_to_color.process(frames);

		if (auto color_frame = frames.get_color_frame())
		{
			cv::Mat InitialImage;
			InitialImage = CameraHelperFunctions::RS_frame_to_mat(color_frame);
			cv::Mat RGBA;
			cv::cvtColor(InitialImage, RGBA, cv::COLOR_RGB2RGBA);
			cv::Mat UndistortRGBA;
			if (bUndistort)
				UndistortRGBA = CameraHelperFunctions::Undistord_CPU(RGBA, m_sColorIntrinsicData);
			else
				RGBA.copyTo(UndistortRGBA);
			if (m_bRotate)
				cv::flip(UndistortRGBA, UndistortRGBA, -1);
			UndistortRGBA.copyTo(TmpRGBATexture.m_mRGBA);
			TmpRGBATexture.m_vImageSize.m_iHeight = UndistortRGBA.size().height;
			TmpRGBATexture.m_vImageSize.m_iWidth = UndistortRGBA.size().width;
		}
	}
	else
	{
		TmpRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
		TmpRGBATexture.m_vImageSize.m_iHeight = 1080;
		TmpRGBATexture.m_vImageSize.m_iWidth = 1920;
	}

	return TmpRGBATexture;
}

CameraIntrinsicData RealSenseBase::GetCameraIntrinsic()
{
	CameraIntrinsicData TmpCameraIntrinsicData;
	if (m_bAllowRS)
	{
		if (!m_bColorIntrinsicData)
			GetColorIntrinsicData();
		TmpCameraIntrinsicData = m_sColorIntrinsicData;
	}
	return TmpCameraIntrinsicData;
}

DepthTexture RealSenseBase::GetDepthMap(bool AutoStop /*= true*/)
{
	DepthTexture TmpDepthTexture;
	TmpDepthTexture.m_mDepth = cv::Mat::zeros(cv::Size(1920, 1080), CV_16FC1);
	TmpDepthTexture.m_vImageSize.m_iHeight = 1080;
	TmpDepthTexture.m_vImageSize.m_iWidth = 1920;

	if (m_bAllowRS)
	{
		rs2::align align_to_color(RS2_STREAM_COLOR);
		auto frames = m_rPipe.wait_for_frames();
		frames.apply_filter(align_to_color);
		frames = align_to_color.process(frames);

		if (auto depth_frame = frames.get_depth_frame())
		{
			auto pf = depth_frame.get_profile().as<rs2::video_stream_profile>();
			cv::Mat depth_raw = cv::Mat(pf.height(), pf.width(), CV_16SC1, const_cast<void*>(depth_frame.get_data()));
			depth_raw.convertTo(TmpDepthTexture.m_mDepth, CV_32FC1);
			TmpDepthTexture.m_mDepth *= depth_frame.get_units();
			TmpDepthTexture.m_vImageSize.m_iWidth = pf.width();
			TmpDepthTexture.m_vImageSize.m_iHeight = pf.height();
			if (m_bRotate)
				cv::flip(TmpDepthTexture.m_mDepth, TmpDepthTexture.m_mDepth, -1);
		}
	}
	
	return TmpDepthTexture;
}

RGBATexture RealSenseBase::GetHeatMap(bool bAutoStop /*= true*/)
{
	RGBATexture TmpRGBATexture;
	TmpRGBATexture.m_mRGBA = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC4);
	TmpRGBATexture.m_vImageSize.m_iHeight = 1080;
	TmpRGBATexture.m_vImageSize.m_iWidth = 1920;

	if (m_bAllowRS)
	{
		rs2::align align_to_color(RS2_STREAM_COLOR);
		auto frames = m_rPipe.wait_for_frames();
		frames.apply_filter(align_to_color);
		frames = align_to_color.process(frames);

		if (auto depth_frame = frames.get_depth_frame())
		{
			DepthTexture TmpDepthTexture;

			auto pf = depth_frame.get_profile().as<rs2::video_stream_profile>();
			cv::Mat depth_raw = cv::Mat(pf.height(), pf.width(), CV_16SC1, const_cast<void*>(depth_frame.get_data()));
			depth_raw.convertTo(TmpDepthTexture.m_mDepth, CV_32FC1);
			TmpDepthTexture.m_mDepth *= depth_frame.get_units();
			TmpDepthTexture.m_vImageSize.m_iWidth = pf.width();
			TmpDepthTexture.m_vImageSize.m_iHeight = pf.height();
			if (m_bRotate)
				cv::flip(TmpDepthTexture.m_mDepth, TmpDepthTexture.m_mDepth, -1);
			
			TmpRGBATexture = CameraHelperFunctions::CreateHeatMap(TmpDepthTexture);
		}
	}

	return TmpRGBATexture;
}

std::vector<float> RealSenseBase::GetPointsCloud(bool AutoStop /*= true*/)
{
	std::vector<float> TmpPointCloud;
	TmpPointCloud.clear();

	if (m_bAllowRS)
	{
		rs2::pointcloud pc;
		rs2::points points;

		rs2::align align_to_color(RS2_STREAM_COLOR);

		auto frames = m_rPipe.wait_for_frames();
		frames = align_to_color.process(frames);
		auto depth = frames.get_depth_frame();
		auto color = frames.get_color_frame();

		if (!color)
			color = frames.get_infrared_frame();

		pc.map_to(color);
		points = pc.calculate(depth);

		std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

		auto sp = points.get_profile().as<rs2::video_stream_profile>();
		auto Vertex = points.get_vertices();

		float FlipFactor = (m_bRotate) ? -1.0f : 1.0f;

		for (int i = 0; i < points.size(); i++)
		{
			if (fabs(Vertex[i].z) <= 0.000001)
				continue;

			TmpPointCloud.push_back(FlipFactor * Vertex[i].x);
			TmpPointCloud.push_back(FlipFactor * Vertex[i].y);
			TmpPointCloud.push_back(Vertex[i].z);
		}
	}

	return TmpPointCloud;
}

void RealSenseBase::GetColorIntrinsicData()
{
	if (!m_bColorIntrinsicData)
	{
		if (m_bAllowRS)
		{
			auto const RGB_Intrinsics = m_rPipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
			m_sColorIntrinsicData.m_vFocalLength.m_fFx = RGB_Intrinsics.fx;
			m_sColorIntrinsicData.m_vFocalLength.m_fFy = RGB_Intrinsics.fy;
			m_sColorIntrinsicData.m_vPrinciplePoint.m_fCx = RGB_Intrinsics.ppx;
			m_sColorIntrinsicData.m_vPrinciplePoint.m_fCy = RGB_Intrinsics.ppy;
			m_sColorIntrinsicData.m_vImageSize.m_iWidth = RGB_Intrinsics.width;
			m_sColorIntrinsicData.m_vImageSize.m_iHeight = RGB_Intrinsics.height;
			m_sColorIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.push_back(RGB_Intrinsics.coeffs[0]);
			m_sColorIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.push_back(RGB_Intrinsics.coeffs[1]);
			m_sColorIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.push_back(RGB_Intrinsics.coeffs[4]);
			m_sColorIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP.push_back(RGB_Intrinsics.coeffs[2]);
			m_sColorIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP.push_back(RGB_Intrinsics.coeffs[3]);
			m_sColorIntrinsicData.m_vDistorsion.m_vDistortionCenter.m_fCx = RGB_Intrinsics.ppx;
			m_sColorIntrinsicData.m_vDistorsion.m_vDistortionCenter.m_fCy = RGB_Intrinsics.ppy;
			m_bColorIntrinsicData = true;
		}		
	}
}

void RealSenseBase::GetIRIntrinsicData()
{
	if (!m_bIrIntrinsicData)
	{
		if (m_bAllowRS)
		{
			auto const IR_Intrinsics = m_rPipe.get_active_profile().get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>().get_intrinsics();
			m_sIrIntrinsicData.m_vFocalLength.m_fFx = IR_Intrinsics.fx;
			m_sIrIntrinsicData.m_vFocalLength.m_fFy = IR_Intrinsics.fy;
			m_sIrIntrinsicData.m_vPrinciplePoint.m_fCx = IR_Intrinsics.ppx;
			m_sIrIntrinsicData.m_vPrinciplePoint.m_fCy = IR_Intrinsics.ppy;
			m_sIrIntrinsicData.m_vImageSize.m_iWidth = IR_Intrinsics.width;
			m_sIrIntrinsicData.m_vImageSize.m_iHeight = IR_Intrinsics.height;
			m_sIrIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.push_back(IR_Intrinsics.coeffs[0]);
			m_sIrIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.push_back(IR_Intrinsics.coeffs[1]);
			m_sIrIntrinsicData.m_vDistorsion.m_vRadialDistortion.m_vK.push_back(IR_Intrinsics.coeffs[4]);
			m_sIrIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP.push_back(IR_Intrinsics.coeffs[2]);
			m_sIrIntrinsicData.m_vDistorsion.m_vTangentialDistortion.m_vP.push_back(IR_Intrinsics.coeffs[3]);
			m_sIrIntrinsicData.m_vDistorsion.m_vDistortionCenter.m_fCx = IR_Intrinsics.ppx;
			m_sIrIntrinsicData.m_vDistorsion.m_vDistortionCenter.m_fCy = IR_Intrinsics.ppy;
			m_bIrIntrinsicData = true;
		}
	}
}
