#ifndef EVALUATION_H
#define EVALUATION_H
#include <cmath>
#include <vector>
#include "GridMap.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


class Evaluation{

public:

	Evaluation():m_fTotalCost(0.0),
		          m_iGroundTNum(0),
		      m_fOutLiearDis(70.0){
	
	};

	int ComputeGroundTruthNum(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllCloud,
		                                                 float f_fOutLiearDis);

	int ComputeGroundTruthNum(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllCloud);

	int ComputeGroundTruthNum(const std::vector<CofidenceValue> & m_vReWardMap);

	float CurrentKnownRate(const std::vector<std::vector<int>> & vGridBoundPsIdx,
		                   const std::vector<std::vector<int>> & vGridTravelPsIdx,
		                      const std::vector<std::vector<int>> & vGridObsPsIdx);
	float CurrentKnownRate(const std::vector<CofidenceValue> & m_vReWardMap);



	float ComputeMovingDis(std::vector<pcl::PointXYZ> & vPathPoints);

	void AddDistanceValue(const float & fPathDis);

	void Output(float & fTotalCost);

private:

	float m_fTotalCost;

	int m_iGroundTNum;//the point number of ground truth (not the ground point) 


	float m_fOutLiearDis;

};



#endif // !EVALUATION_H
