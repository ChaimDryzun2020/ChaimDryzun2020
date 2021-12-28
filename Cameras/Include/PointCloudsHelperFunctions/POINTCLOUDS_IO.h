// POINTCLOUDS_IO.h
// This is the Header file for all kind of point clouds input/ output operations
//
// First created by Chaim Dryzun at 11.04.2021

#pragma once

#include "Constants.h"

namespace POINTCLOUDS_IO
{
	// Transforming a depth map to 3D point cloud 
	std::vector<std::vector<float>> rangeMap_To_point(const float* rangeMap, const unsigned int width, const unsigned int height, const float fx, const float fy, const float cx, const float cy);

	// Transforming a 1D float vector to vector of 3D points
	std::vector<std::vector<float>> points_To_vector(const float* points, const unsigned int size, const unsigned int dim = 3);

	// Transforming a depth map to 3D point cloud in PCL's format
	pcl::PointCloud<pcl::PointXYZ>::Ptr rangeMap_To_pointCloud(const float* rangeMap, const unsigned int width, const unsigned int height, const float fx, const float fy, const float cx, const float cy);

	// Transforming a 1D float vector to vector of 3D points in PCL's format
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_To_pointCloud(const float* points, const unsigned int size);

	std::vector<Eigen::Vector3f> points_To_EigenVector(const float* points, const unsigned int size);
	Eigen::Vector3f vector_To_EigenVector(const float* points, const unsigned int size);
}
