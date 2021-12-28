#pragma once

#include "Constants.h"

class PatternSearch
{
public:

	PatternSearch(void* objp, double (*func)(std::vector<double>, void*));
	virtual ~PatternSearch();

	void SetVariables(const std::vector<double>& Variables);
	void SetStepSize(const std::vector<double>& StepSize);
	void SetEpsilon(double Epsilon);
	void SetMinVal(double MinVal);
	void SetMaxIter(int MaxIter);

	bool Optimize();

	std::vector<double> GetVariables();
	double GetVal();

private:
	void* m_ObjPtr;
	double (*m_Function)(std::vector<double>, void*);

	std::vector<double> m_vVariables;
	std::vector<double> m_vStepSize;
	double m_dVal;
	double m_dEpsilon;
	double m_dMinVal;
	int m_iMaxIter;
	bool m_bMin;

};

