#include "Evaluation.h"


int Evaluation::ComputeGroundTruthNum(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllCloud,float f_fOutLiearDis){

	m_fOutLiearDis = f_fOutLiearDis;

	for (int i = 0; i != pAllCloud->points.size(); ++i){
	
		float fOneDis = sqrt(pow((pAllCloud->points[i].x - 0.0f), 2.0f)
			               + pow((pAllCloud->points[i].y - 0.0f), 2.0f)
			               + pow((pAllCloud->points[i].z - 0.0f), 2.0f));

		if (fOneDis <= m_fOutLiearDis)
			m_iGroundTNum++;
	
	}

	return m_iGroundTNum;

}
int Evaluation::ComputeGroundTruthNum(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllCloud) {

	m_iGroundTNum = pAllCloud->points.size();
	return m_iGroundTNum;

}
int Evaluation::ComputeGroundTruthNum(const std::vector<CofidenceValue> & m_vReWardMap) {

	//count data grid
	for (int i = 0; i != m_vReWardMap.size(); ++i) {
		if(m_vReWardMap[i].iLabel==2)
			m_iGroundTNum++;
	}

	//ground truth number
	return m_iGroundTNum;

}


float Evaluation::CurrentKnownRate(const std::vector<std::vector<int>> & vGridBoundPsIdx,
	                              const std::vector<std::vector<int>> & vGridTravelPsIdx,
	                                 const std::vector<std::vector<int>> & vGridObsPsIdx) {

	int iKnownPointNum = 0;
	for (int i = 0; i != vGridBoundPsIdx.size(); ++i) {
		for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j) {
			iKnownPointNum++;
		}
	}

	for (int i = 0; i != vGridTravelPsIdx.size(); ++i) {
		for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j) {
			iKnownPointNum++;
		}
	}

	for (int i = 0; i != vGridObsPsIdx.size(); ++i) {
		for (int j = 0; j != vGridObsPsIdx[i].size(); ++j) {
			iKnownPointNum++;
		}
	}

	float fKnownRate = float(iKnownPointNum) / float(m_iGroundTNum);
	return fKnownRate;

}
float Evaluation::CurrentKnownRate(const std::vector<CofidenceValue> & m_vReWardMap) {

	float fKnownPointNum = 0.0;

	for (int i = 0; i != m_vReWardMap.size(); ++i) {
	
		if (m_vReWardMap[i].bKnownFlag&&m_vReWardMap[i].iLabel == 2)
			fKnownPointNum = fKnownPointNum + 1.0;
			
	}

	float fKnownRate = fKnownPointNum / float(m_iGroundTNum);
	return fKnownRate;

}

float Evaluation::ComputeMovingDis(std::vector<pcl::PointXYZ> & vPathPoints) {

	float fPathLen = 0.0;
	
	if (!vPathPoints.size())
		return fPathLen;

	for (int i = 0; i < vPathPoints.size() - 1; ++i) {
	
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


