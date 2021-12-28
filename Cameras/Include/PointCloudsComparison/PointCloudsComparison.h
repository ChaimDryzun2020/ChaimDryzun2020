#pragma once

#include <vector>
#include <tuple>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

class PointCloudsComparison
{
public:

	PointCloudsComparison();			// Default constructor
	virtual ~PointCloudsComparison();	// Destructor

	virtual void SetCloudA(float cloud[], int length);
	virtual void SetCloudB(float cloud[], int length);

	virtual void SetCloudA(const std::vector<std::vector<float>>& cloud);
	virtual void SetCloudB(const std::vector<std::vector<float>>& cloud);

	inline virtual void SetCloudA(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud) { pcl::copyPointCloud(*pcl_cloud, *m_vCloudA); }
	inline virtual void SetCloudB(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud) { pcl::copyPointCloud(*pcl_cloud, *m_vCloudB); }

	virtual void SetMaxDistance(float MaxDistance);

	inline virtual void BiDirectionalComparison(bool BiDirectionalflag) { m_bBiDirectional = BiDirectionalflag; }
	inline virtual void SignedDisatnce(bool SignedDisatnceflag) { m_bSignedDistance = SignedDisatnceflag; }

	inline virtual void Reset() { m_bCalc = false; }

	virtual bool Calculate();

	virtual float GetRMSD();
	virtual float GetMAE();
	virtual float GetHausdorff();
	virtual std::vector< std::tuple< int, int, float> > GetLocalDistances();

private:

	float m_fRMSD, m_fMAE, m_fHausdorff;
	float m_fMaxDistance;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_vCloudA;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_vCloudB;
	std::vector< std::tuple< int, int, float> > m_vLocalDistance;
	bool m_bCalc;
	bool m_bBiDirectional;
	bool m_bSignedDistance;
};

