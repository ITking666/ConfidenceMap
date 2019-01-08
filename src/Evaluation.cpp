#include "Evaluation.h"

float Evaluation::ComputeMovingDis(std::vector<pcl::PointXYZ> & vPathPoints) {

	float fPathLen = 0.0;

	for (int i = 0; i != vPathPoints.size() - 1; ++i) {
	
		float fOneLen = sqrt(pow((vPathPoints[i + 1].x - vPathPoints[i].x), 2.0f)
			+ pow((vPathPoints[i + 1].y - vPathPoints[i].y), 2.0f)
			+ pow((vPathPoints[i + 1].z - vPathPoints[i].z), 2.0f));
	
		fPathLen = fPathLen + fOneLen;
	}

	return fPathLen;

}

void Evaluation::AddDistanceValue(const float & fPathDis) {

	m_fTotalCost = m_fTotalCost + fPathDis;

}

void Evaluation::Output(float & fTotalCost){

	fTotalCost =  m_fTotalCost;

}

