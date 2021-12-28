#pragma once

#include "Constants.h"

namespace PointCloudDownSampling
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr DownsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float voxel_grid_size = -1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, float voxel_grid_size = -1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr MedianDownSampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr MedianDownSampling(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, float voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZ>::Ptr MedianDownSampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float voxel_grid_size, const Eigen::Vector3f& PointA, const Eigen::Vector3f& PointB, const float Scale = 1.51);
	pcl::PointCloud<pcl::PointXYZ>::Ptr MedianDownSampling(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, std::vector<float>& weights, float voxel_grid_size);
}

