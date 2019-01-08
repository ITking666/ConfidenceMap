#ifndef EVALUATION_H
#define EVALUATION_H
#include <cmath>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


class Evaluation{

public:

	Evaluation():m_fTotalCost(0.0){
	
	};

	float ComputeMovingDis(std::vector<pcl::PointXYZ> & vPathPoints);

	void AddDistanceValue(const float & fPathDis);

	void Output(float & fTotalCost);

private:

	float m_fTotalCost;

};



#endif // !EVALUATION_H
