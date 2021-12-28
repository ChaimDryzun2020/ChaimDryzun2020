#pragma once

#include "Constants.h"

class KABSCH
{
public:

	KABSCH();
	virtual ~KABSCH();

	virtual void SetTargetCloud(float vertices[], int length);
	virtual void SetSourceCloud(float vertices[], int length);
	virtual void SetWeights(float weights[], int length);

	virtual void SetTargetCloud(const std::vector<std::vector<float>>& vertices);
	virtual void SetSourceCloud(const std::vector<std::vector<float>>& vertices);
	virtual void SetWeights(const std::vector<float>& weights);

	virtual void SetTargetCloud(const Eigen::MatrixX3f& vertices);
	virtual void SetSourceCloud(const Eigen::MatrixX3f& vertices);
	virtual void SetWeights(const Eigen::VectorXf& weights);

	virtual bool Align();

	virtual float GetScore();

	virtual std::vector<std::vector<float>> GetTransformedCloud();
	virtual Eigen::MatrixX3f GetSourceCloud();

	virtual Eigen::Matrix4f GetTransformation();

private:

	Eigen::MatrixX3d m_vTarget;
	Eigen::MatrixX3d m_vSource;
	Eigen::VectorXd m_vWeights;

	double m_fScore;
	Eigen::Matrix4d m_mTransformation;

};
