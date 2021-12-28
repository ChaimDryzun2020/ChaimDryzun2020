// Constants.h
// Contains general declarations and definitions applied for all the files 
//
// Contains general header files for Eigen, PCL,  and windows
// Contains conversion factor definitions
// Contains constants for calculations. 
//
// First created by Chaim Dryzun at 11.04.2021

#pragma once

// General windows header files
#include <limits>
#include <ctime>
#include <memory>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen header file
#include <Eigen/Dense>

// General PCL header files
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// Constant and Conversion factor definitions

// Angle conversion factor definitions
#define DEG_TO_RAD 0.01745329251994329576923690768489
#define RAD_TO_DEG 57.295779513082320876798154814105

// Distance constants and Conversion factor definitions
#define DISTANCE_FACTOR 1000.0
#define M_TO_MM 1000.0
#define MM_TO_M 0.001
#define M_TO_CM 100.0
#define CM_TO_M 0.01

// PI definitions
#define PI 3.1415926535897932384626433832795
#define TWOPI 6.283185307179586476925286766559

// Zero definition
#define ZEPS 0.0000000001
#define E_MINUS_NINE 0.000000001
#define POINTS_SENSITIVITY 0.000001

// Infinity definition
#define INFTY 10000000000.0

// Camera min and max distance constants
#define CAM_MIN_DIST 0.2
#define CAM_MAX_DIST 9.0

// Min ROI constants
#define MIN_X_ROI -5000.0
#define MIN_Y_ROI -5000.0
#define MIN_Z_ROI 0.0

// Max ROI constants
#define MAX_X_ROI 5000.0
#define MAX_Y_ROI 5000.0
#define MAX_Z_ROI 15000.0

// Error float value
#define FLOAT_ERROR -1.0

// Minimal number of points in a valid point cloud
#define MIN_CLOUD_NUM 4

//  Minimal resolution
#define MIN_RESOLUTION 0.0001

