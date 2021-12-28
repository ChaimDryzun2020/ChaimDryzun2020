// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#pragma once

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#pragma warning (disable:4786)
#pragma warning (disable:4101)

#include <math.h>
#include <float.h>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <crtdbg.h>
#include <algorithm>
#include <utility>
#include <string>
#include <iomanip>
#include <sstream>
#include <omp.h>
#include <iostream>
#include <fstream>
#include <cderr.h>
#include <tuple>
#include <limits>

#include <windows.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#define ZEPS 0.0000000001

// Microsoft Visual C++ will insert additional declarations immediately before the previous line.


