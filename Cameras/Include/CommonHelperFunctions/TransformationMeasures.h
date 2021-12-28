// TransformationMeasurs.h - Contains declarations of the transformation measurs
// This is the Header file for functions exposed outside
//
// First created by Denise Bishevsky at 25/10/2021

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <utility>

class TransformationMeasures 
{
public:

	// Default constructor
	TransformationMeasures();

	// Constructor with two given transformations
	TransformationMeasures(Eigen::Matrix4f mTransformA, Eigen::Matrix4f mTransformB);

	// Default destructor
	~TransformationMeasures();

	// Set the transformations
	void SetTransformations(Eigen::Matrix4f mTransformA, Eigen::Matrix4f mTransformB);
	// Get the euclidian distance
	float GetEuclidianDistance();
	// Get the euclidian distance of x, y, z axis separately
	Eigen::Vector3f GetEuclidianDistanceByAxis();
	// Get the angular diffrence angle
	float GetAngularDiffrenceAngle();
	// Get the angular diffrence distance
	float GetAngularDiffrenceDistance();
	// Get the angular diffrence distance of x, y, z axis separately
	Eigen::Vector3f GetAngularDiffrenceDistanceByAxis();
	// Get the transformation matrix distance
	float GetTransformationMatrixDistance();
	// Calculate Angular iffrence Distance
	float CalculateAngularDiffrenceDistance(Eigen::Vector3f vVector, Eigen::Matrix3f mCombinedRotation);
	float CalculateAngularDiffrenceDistance(Eigen::Vector4f vVector, Eigen::Matrix4f mCombinedRotation);



private:
	// Claculate the distance, the angular diffrence angle, angular diffrence distance and transformation matrix distance
	void Calculate();
	// Extract the rotation matrix and the translation vector from the transformation matrix
	std::pair<Eigen::Vector3f, Eigen::Matrix3f> ExtractTranslationRotation(Eigen::Matrix4f mInputMatrix);
	// Calculate the combined transformation
	Eigen::Matrix4f CalculateCombinedTransformation();
	// Calculate the combined rotation
	Eigen::Matrix3f CalculateCombinedRotation(Eigen::Matrix3f mRotationA, Eigen::Matrix3f mRotationB);


	Eigen::Matrix4f m_mTransformA, m_mTransformB;	// The transformation we want to compare

	float m_fEuclidianDistance;		// The euclidian distance between two vectors
	Eigen::Vector3f m_vEuclidianDistanceByAxis;	// The euclidian distance between two vectors of x, y, z axis separately
	float m_fAngularDiffrenceAngle;  // The angular diffrence angle of the two transformations
	float m_fAngularDiffrenceDistance;  // The angular diffrence distance of the two transformations
	Eigen::Vector3f m_vAngularDiffrenceDistanceByAxis;	// The angular diffrence distance of the two transformations of x, y, z axis separately
	float m_fTransformationMatrixDistance; // The transformation matrix distance of the whole two transformations 
	bool m_bCalc; // a bool that will indicate if we already calculated all the distances 

};