#pragma once

#include "Constants.h"
#include "Kabsch.h"
#include <pcl/kdtree/kdtree_flann.h>

enum ICP_Type
{
	PointToPoint = 0,
	PointToSurface,
	SurfaceToPoint,
	SurfaceToSurface,
	IcpTypesNum
};

enum Optimization_Type
{
	RMSD = 0,
	MAE,
	OptTypeNum
};

class GeneralICP
{
public:

	GeneralICP();
	virtual ~GeneralICP();

	virtual void SetTargetCloud(float vertices[], int length);
	virtual void SetSourceCloud(float vertices[], int length);
	virtual void SetTargetWeights(float weights[], int length);

	virtual void SetTargetCloud(const std::vector<std::vector<float>>& vertices);
	virtual void SetSourceCloud(const std::vector<std::vector<float>>& vertices);
	virtual void SetTargetWeights(const std::vector<float>& weights);

	virtual void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud);
	virtual void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_cloud);

	virtual void SetTargetNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr& pcl_normals);
	virtual void SetSourceNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr& pcl_normals);

	virtual void SetDistanceMin(float DistanceMin);
	virtual void SetRotationAngleMin(float RotationAngleMin);
	virtual void SetMinDiffernce(float MinDifference);

	virtual void SetICP_Type(unsigned int ICPtype);
	virtual void SetOptimizationType(unsigned int OptType);

	virtual void SetLogFileName(std::string LogFileName);
	virtual void SaveLog(bool SaveLg);

	virtual void UseWeights(bool UseWeight);

	virtual bool Align();

	virtual float GetScore();

	virtual void SetMaxIterNum(unsigned int MaxIterNum);
	virtual void SetMaxCorrespondenceDistance(float MaxCorrespondenceDistance);

	virtual std::vector<std::vector<float>> GetTransformedCloud();
	virtual Eigen::Matrix4f GetTransformation();


private:

	typedef struct LocalRigidTransform
	{
		float m_vTranslation[3];
		float m_vEulerRotation[3];
		float m_fRMSD;

		float Norm()
		{
			float Nrm = 0.0f;
			for (int i = 0; i < 3; i++)
			{
				Nrm += (m_vTranslation[i] * m_vTranslation[i]);
				Nrm += (m_vEulerRotation[i] * m_vEulerRotation[i]);
			}
			return sqrt(Nrm / 6.0);
		}
	} LocalRigidTransform;


	virtual void FindCorrespondances(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source);
	virtual void FindCorrespondances(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source, std::vector<float>& Weights);
	virtual void UpdateMaxDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source);

	virtual bool CalculateTransformSVD(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source);
	virtual bool CalculateTransformSVD(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source, std::vector<float>& Weights);
	virtual bool CalculateTransformPCLsICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source);
	virtual bool CalculateTransformPatternSearch();
	virtual void Extrapolate(pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, pcl::PointCloud<pcl::PointXYZ>::Ptr& Source);

	double CalcRMSDforTransform(std::vector<double> trnsfrm);
	static double OptimizationTargetFunction(std::vector<double> trnsfrm, void* ObjPtr)
	{
		return ((GeneralICP*)ObjPtr)->CalcRMSDforTransform(trnsfrm);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_vTarget;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_vSource;

	pcl::PointCloud<pcl::Normal>::Ptr m_vTargetNormals;
	pcl::PointCloud<pcl::Normal>::Ptr m_vSourceNormals;

	std::vector<float> m_vTargetWeight;

	float m_fScore;
	float m_fRMSD;
	float m_fRotationAngle;
	float m_fTranslation;

	float m_fMaxCorrespondenceDistance;

	float m_fRMSD_min;
	float m_fAngleMin;
	float m_fDiffMin;

	Eigen::Matrix4f m_mTransformation;

	KABSCH m_clssKabsch;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
	std::vector<LocalRigidTransform> m_vT;
	bool m_bAligned;
	bool m_bConverg;
	ICP_Type m_eICPtype;
	Optimization_Type m_eOptType;

	std::string m_sLogFileName;
	bool m_bSaveLog;
	bool m_bWeights;

	unsigned int m_iMaxIterNum;
};


