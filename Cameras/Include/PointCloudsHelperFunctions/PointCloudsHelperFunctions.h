#pragma once

#include "Constants.h"

namespace PointCloudsHelperFunctions
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr VectorToPCL(const std::vector<std::vector<float>>& Cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr VectorToPCL(const std::vector<std::vector<float>>& Cloud, const std::vector<std::vector<int>>& Colors);
	std::vector<std::vector<float>> PCLToVector(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud);
	void PCLToVector(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pcl_cloud, std::vector<std::vector<float>>& Cloud, std::vector<std::vector<int>>& Colors);
	float ComputeBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud, std::vector<float>& upper_right, std::vector<float>& bottom_left, float factor = 1.0);
	float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	float computeDistanceBetweenClouds(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudA, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudB);
	std::vector<std::vector<float>> CropCloud(const std::vector<std::vector<float>>& Cloud, const Eigen::Vector3f& PointA, const Eigen::Vector3f& PointB, const float Scale = 1.51);
	pcl::PointCloud<pcl::PointXYZ>::Ptr CropCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud, const Eigen::Vector3f& PointA, const Eigen::Vector3f& PointB, const float Scale = 1.51);
	pcl::PointCloud<pcl::PointXYZ>::Ptr CropCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud, std::vector<float>& weights, const Eigen::Vector3f& PointA, const Eigen::Vector3f& PointB, const float Scale = 1.51);
	pcl::PointCloud<pcl::PointXYZ>::Ptr EmptyCloud();
}



