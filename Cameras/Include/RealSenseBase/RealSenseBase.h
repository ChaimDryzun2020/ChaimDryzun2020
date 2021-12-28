#pragma once

#include "CameraBaseImplementation.h"

#include <librealsense2/rs.hpp>

class RealSenseBase : public CameraBaseImplementation
{
protected:
	std::string m_sSerial;
	rs2::pipeline m_rPipe;	
	bool m_bAllowRS;

	CameraIntrinsicData m_sColorIntrinsicData;
	bool m_bColorIntrinsicData;
	CameraIntrinsicData m_sIrIntrinsicData;
	bool m_bIrIntrinsicData;

public:
	RealSenseBase();
	virtual ~RealSenseBase() { /*m_rPipe.stop();*/ }

	virtual void SetPipe(rs2::pipeline& rPipe);

	virtual int GetCameraType() { return CamerasEnum::REALSENSE_BASE; }

	virtual bool SupportsRGB() { return true; }

	virtual bool SetRGBATexture(RGBATexture& mRGBATexture, bool bUndistort = true);
	virtual bool RGBStream(bool bStreamRGB) { m_bStreamRGB = bStreamRGB;  return true; }
	virtual RGBATexture UpdateRGBAImage(bool bUndistort = true);

	CameraIntrinsicData GetCameraIntrinsic();

	virtual DepthTexture GetDepthMap(bool AutoStop = true);
	virtual RGBATexture GetHeatMap(bool bAutoStop = true);
	virtual std::vector<float> GetPointsCloud(bool AutoStop = true);

protected:
	void GetColorIntrinsicData();
	void GetIRIntrinsicData();
};

typedef CSmartPtr<RealSenseBase> RealSenseBasePtr;