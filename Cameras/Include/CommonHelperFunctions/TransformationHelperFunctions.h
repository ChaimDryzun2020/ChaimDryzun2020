// Transformationhelper.h - Contains declarations of the transformation helper
// This is the Header file for functions exposed outside
//
// First created by Denise Bishevsky at 25/10/2021

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <unsupported/Eigen/EulerAngles>

namespace TransformationHelperFunctions
{
	// Transferring rotation matrix to euler 
	Eigen::Vector3f RotationMatrixToEulerAngles(const Eigen::Matrix3f& mRotationMatrix, const bool bOutputInDeg = false);

	// Transferring rotation matrix to euler 
	Eigen::EulerAnglesXYZf RotationMatrixToEuler(const Eigen::Matrix3f &mRotationMatrix);

	// Transferring euler rotation to rotation matrix
	Eigen::Matrix3f EulerToRotationMatrix(const float fX, const float fY, const float fZ, const bool bFromUnity = false);

	// Transferring euler rotation to rotation matrix
	Eigen::Matrix3f EulerAnglesToRotationMatrix(const Eigen::Vector3f& vEulerAngles, const bool bFromUnity = false, const bool bInputInDeg = false);

	// Transferring rotation matrix to quaternion 
	Eigen::Quaternion<float> RotationMatrixToQuaternion(const Eigen::Matrix3f &mRotationMatrix);

	// Transferring quaternion rotation to rotation matrix
	Eigen::Matrix3f QuaternionToRotationMatrix(const Eigen::Quaternion<float> &qQuaternion);

	// Transferring rotation matrix to angle axis 
	Eigen::AngleAxisf RotationMatrixToAngleAxis(const Eigen::Matrix3f& mRotationMatrix);

	// Transferring angle axis rotation to rotation matrix
	Eigen::Matrix3f AngleAxisToRotationMatrix(const Eigen::AngleAxisf& aAngleAxis);
	
	// Transferring data obtained from Unity to a transformation matrix
	Eigen::Matrix4f CreateTransformationMatrix(const Eigen::Vector3f& vTranslation, const Eigen::Vector3f& vEulerAngles);

	// Apply a given transformation on a set of points
	std::vector<std::vector<float>> ApplyTransformation(const std::vector<std::vector<float>>& vPoints, const Eigen::Matrix4f& mTransformation);
}
