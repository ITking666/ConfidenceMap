#ifndef LOCALPATHOPTIMIZATION_H
#define LOCALPATHOPTIMIZATION_H
#include "Confidence.h"
#include "TSP.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>


struct QualityPair{

	int idx;
	float quality;

	QualityPair(){
		idx = 0;
		quality = 0.0;
	};

};


class PathOptimization{

public:
	
	//
	PathOptimization():m_pControlCloud(new pcl::PointCloud<pcl::PointXY>){
	
	};

	//sort the controls from max to min
	std::vector<QualityPair> & SortFromBigtoSmall();

	//compute the center of control grid
	void GetControlCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
		                  const std::vector<std::vector<int>> & vGridPointIdx);

	////compute the moving distance
	static pcl::PointXY MovingDistance(const pcl::PointXY & oLinePoint,
		const pcl::PointXY & oObsPoint,
		const float & fMovingDis);

	////generate new local path
	void NewLocalPath(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
		                                            GridMap & oMap,
		                                      float fMoveDis = 1.0,
		                                    int iMaxSearchTime = 7,
		                                    int iMinPathLength = 5);

	std::vector<QualityPair> m_vControls; 
	pcl::PointCloud<pcl::PointXY>::Ptr m_pControlCloud;

	//construct a 2D point clouds

private:


};




#endif // !LocalPathOptimization_h

