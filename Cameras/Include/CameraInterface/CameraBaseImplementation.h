#pragma once

#include "CameraInterface.h"
#include "CamerasEnum.h"

class CameraBaseImplementation : public CameraInterface
{
protected:
	std::string m_sID, m_sName, m_sType;
	bool m_bRotate;
	bool m_bStreamRGB;
	
public:
	CameraBaseImplementation();
	virtual ~CameraBaseImplementation() { }

	virtual int GetCameraType() { return CamerasEnum::NULL_CAMERA; }

	virtual std::string GetID() { return m_sID; }
	virtual std::string GetName() { return m_sName; }
	virtual std::string GetType() { return m_sType; }

	virtual void SetID(const std::string& sID) { m_sID = sID;  }
	virtual void SetName(const std::string& sName) { m_sName = sName; }
	virtual void SetType(const std::string& sType) { m_sType = sType;  }

	virtual void Rotate180(bool bRotate = false) { m_bRotate = bRotate; }

	virtual bool SupportsRGB() { return false; }

	virtual bool SetRGBATexture(RGBATexture& mRGBATexture, bool bUndistort = true);
	virtual bool RGBStream(bool bStreamRGB) { return true; }
	virtual RGBATexture UpdateRGBAImage(bool bUndistort = true);

	virtual CameraIntrinsicData GetCameraIntrinsic();

	virtual DepthTexture GetDepthMap(bool bAutoStop = true);
	virtual RGBATexture GetHeatMap(bool bAutoStop = true);
	virtual std::vector<float> GetPointsCloud(bool bAutoStop = true);
};
