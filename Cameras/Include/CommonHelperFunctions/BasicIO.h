// BasicIO.h
// This is the Header file for all kind of input output operations
//
// First created by Chaim Dryzun at 5.12.2021

#pragma once

#include "Constants.h"
#include <string>
#include <iostream>
#include <map>

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"


namespace BASIC_IO
{
	// Saving one log line: time - Test - Value
	void Log(const std::string& sFileName, const std::string& sText, const float fValue = -100000.0);

	void LoadCameraIntrinsicJSON(const std::string CameraFilename, bool bIR, float& cx, float& cy, float& fx, float& fy, int& Height, int& Width);

	void LoadRGBJSON(const std::string RGBFilename, cv::Mat& RGBImage);

	void LoadAdditionalData(const std::string AdditionalDataFilename, float& RMSD, std::string& Text, std::map<int, bool>& PartsAssemblyState);

	void LoadDepthJSON(const std::string DepthFilename, cv::Mat& DepthImage, float& MinVal, float& MaxVal);
}
