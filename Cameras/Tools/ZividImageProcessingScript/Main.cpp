#include "DialogBoxes.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <crtdbg.h>
#include <algorithm>
#include <utility>
#include <string>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cderr.h>
#include <filesystem>
namespace fs = std::filesystem;

#include <json/json.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>

int main(int argc, char* argv[])
{
	std::string ext = "ply";
	std::string PclFileName;

	// If there is one argument or more, the first argument is the input PLY file
	// If not, open a dialog box so the user can choose the input image file
	if (argc > 1)
		PclFileName = argv[1];
	else
		PclFileName = DialogBoxes::LoadFileDialogBox(ext);

	// Getting the input image file
	ext = "png";
	std::string InputRGBFileName;

	// If there is two argument or more, the second argument is the input image file
	// If not, open a dialog box so the user can choose the input image file
	if (argc > 2)
		InputRGBFileName = argv[2];
	else
		InputRGBFileName = DialogBoxes::LoadFileDialogBox(ext);

	// Default JSON intrinsic file name
	ext = "json";
	std::string InputJSON = "Intrinsics.json";

	// If there are two argument or more, the second is the intrinsic JSON file
	if (argc > 3)
		InputJSON = argv[3];
	else
		InputJSON = DialogBoxes::LoadFileDialogBox(ext);

	std::cout << "\n\tReading PLY file\n";

	// Define color XYZ cloud container
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Read the PLY file
	pcl::io::loadPLYFile(PclFileName, *PCL_cloud);
	std::cout << "\tReading PLY file - Done\n";

	// Get The PLY file's name
	fs::path path = PclFileName;
	std::string Name = (path.stem()).u8string();	

	// Set the output PLY name
	std::string OutputPclFileName = "C:\\PlyFile\\PLY.ply";

	std::cout << "\tSaving ASCII PLY file\n";

	// Save the output ASCII PLY
	pcl::io::savePLYFile(OutputPclFileName, *PCL_cloud, false);

	std::cout << "\tSaving ASCII PLY file - Done\n";

	// Initial Hard coded default data for the intrinsics of the ZIVID camera
	double CX = 960.339721679688;
	double CY = 626.373474121094;
	double FX = 2766.52807617188;
	double FY = 2769.16943359375;
	double K1 = -0.261241167783737;
	double K2 = 0.242411836981773;
	double K3 = 0.21816785633564;
	double P1 = 0.000792709703091532;
	double P2 = -0.00100542674772441;

	std::cout << "\tReading intrinsic JSON file\n";

	// Trying to open the intrinsic JSON file
	Json::Value IntrinsicsJsonData;
	std::ifstream IntrinsicsTest(InputJSON, std::ifstream::binary);
	Json::CharReaderBuilder JsonIntrinsicsReader;
	JSONCPP_STRING IntrinsicsErrs;

#if defined(_DEBUG)
	JSONCPP_STRING IntrinsicsErrsA;
	if (!Json::parseFromStream(JsonIntrinsicsReader, IntrinsicsTest, &IntrinsicsJsonData, &IntrinsicsErrsA))
	{
		std::cout << "\tCould not parse depth input JSON" << std::endl;
		std::cout << "\tUsing hard coded values\n";
		//return 1;
	}
	//errs = "";
#else
	if (!Json::parseFromStream(JsonIntrinsicsReader, IntrinsicsTest, &IntrinsicsJsonData, &IntrinsicsErrs))
	{
		std::cout << "\tCould not parse input depth JSON" << std::endl;
		std::cout << "\tUsing hard coded values\n";
		//std::cout << IntrinsicsErrs << std::endl;
		//return 1;
	}
	else
	{
		// Reading the intrinsic data from the JSON file
		CX = IntrinsicsJsonData["CX"].asDouble();
		CY = IntrinsicsJsonData["CY"].asDouble();
		FX = IntrinsicsJsonData["FX"].asDouble();
		FY = IntrinsicsJsonData["FY"].asDouble();
		K1 = IntrinsicsJsonData["K1"].asDouble();
		K2 = IntrinsicsJsonData["K2"].asDouble();
		K3 = IntrinsicsJsonData["K3"].asDouble();
		P1 = IntrinsicsJsonData["P1"].asDouble();
		P2 = IntrinsicsJsonData["P2"].asDouble();
	}
#endif

	std::cout << "\tReading intrinsic - Done\n";

	// Setting the Distortion coefficients and the camera matrix
	cv::Mat distortionCoefficients(1, 5, CV_64FC1, cv::Scalar(0));
	cv::Mat cameraMatrix(3, 3, CV_64FC1, cv::Scalar(0));

	cameraMatrix.at<double>(0, 0) = FX;
	cameraMatrix.at<double>(0, 2) = CX;
	cameraMatrix.at<double>(1, 1) = FY;
	cameraMatrix.at<double>(1, 2) = CY;
	cameraMatrix.at<double>(2, 2) = 1.0;

	distortionCoefficients.at<double>(0, 0) = K1;
	distortionCoefficients.at<double>(0, 1) = K2;
	distortionCoefficients.at<double>(0, 2) = P1;
	distortionCoefficients.at<double>(0, 3) = P2;
	distortionCoefficients.at<double>(0, 4) = K3;

	std::cout << "\tReading RGB file\n";

	// Reading the input image
	cv::Mat image = cv::imread(InputRGBFileName);

	std::cout << "\tReading RGB file - Done\n";

	cv::Mat imageUndistorted;

	std::cout << "\tPerforming Undistort\n";

	// Undistorting the image
	cv::undistort(image, imageUndistorted, cameraMatrix, distortionCoefficients);

	std::cout << "\tPerforming Undistort - Done\n";

	// Set the output RGB file name
	std::string OutputRGBFileName = "C:\\PlyFile\\RGB.png";

	std::cout << "\tSaving RGB file\n";

	// Saving the undistorted image
	cv::imwrite(OutputRGBFileName, imageUndistorted);
	std::cout << "\tSaving RGB file - Done\n";
	std::cout << "\nProgram ended successfully\n";


	return 0;
}