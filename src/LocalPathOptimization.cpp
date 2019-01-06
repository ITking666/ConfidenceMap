#include "LocalPathOptimization.h"

std::vector<QualityPair> & PathOptimization::SortFromBigtoSmall(){

	for (int i = 0; i < m_vControls.size(); i++)
	{
		QualityPair oTemp;
		for (int j = i + 1; j < m_vControls.size(); j++){

			if (m_vControls[i].quality < m_vControls[j].quality){
				//exchange
				oTemp.quality = m_vControls[i].quality;
				oTemp.idx = m_vControls[i].idx;
				//
				m_vControls[i].idx = m_vControls[j].idx;
				m_vControls[i].quality = m_vControls[j].quality;
				//
				m_vControls[j].idx = oTemp.idx;
				m_vControls[j].quality = oTemp.quality;
			}
		}
	}

	return m_vControls;

}

void PathOptimization::GetControlCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
                                        const std::vector<std::vector<int>> & vGridPointIdx) {

	m_pControlCloud->points.clear();
	for (int i = 0; i != m_vControls.size(); ++i) {

		//get the 3d center
		pcl::PointXYZ oControlCenter =
			TSP::ComputeCentersPosition(pCloud, vGridPointIdx, m_vControls[i].idx);

		//only record x,y
		pcl::PointXY oPoint;
		oPoint.x = oControlCenter.x;
		oPoint.y = oControlCenter.y;
		m_pControlCloud->points.push_back(oPoint);
	}

}

void PathOptimization::NewLocalPath(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	                                                              GridMap & oMap,
	                                                              float fMoveDis,
	                                                          int iMaxSearchTime,
	                                                          int iMinPathLength){

	pcl::PointCloud<pcl::PointXY>::Ptr pLineCloud(new pcl::PointCloud<pcl::PointXY>);
	std::vector<int> vLinePointStatus(pCloud->points.size(), -1);
	std::vector<pcl::PointXYZ> vNewAncherPoints;

	//construct a xy point clouds for line input
	for (int i = 0; i != pCloud->points.size(); ++i) {
		pcl::PointXY oXYPoint;
		oXYPoint.x = pCloud->points[i].x;
		oXYPoint.y = pCloud->points[i].y;
		pLineCloud->points.push_back(oXYPoint);
	}
	std::cout << "total line number: " << pLineCloud->points.size() << std::endl;
	//construct a kdtree
	pcl::KdTreeFLANN<pcl::PointXY> oLineTree;
	oLineTree.setInputCloud(pLineCloud);

	int iQBoundIdx = 0;
	int iQSuccess = 0;//The successed seed
	
	//if the distance is too short
	if (m_pControlCloud->points.size() < iMinPathLength)
		return;

	if (m_pControlCloud->points.size() < iMaxSearchTime)
		iMaxSearchTime = m_pControlCloud->points.size();

	
	//compute the index
	while (iQBoundIdx < iMaxSearchTime && iQSuccess < 3) {

		std::vector<int> vSearchIdx;
		std::vector<float> vSearchDis;
		//search distance of center of sign based on max distance to center
		std::cout << "m_pControlCloud->points[iQBoundIdx]: " << m_pControlCloud->points[iQBoundIdx] << std::endl;
		std::cout << "m_pControlCloud->points[iQBoundIdx]: " << m_pControlCloud->points[iQBoundIdx] << std::endl;
		oLineTree.nearestKSearch(m_pControlCloud->points[iQBoundIdx], 1, vSearchIdx, vSearchDis);
		std::cout << "vSearchIdx[0]: " << vSearchIdx[0] << std::endl;
		//if the searched point is first point, which indicates the searched point is behind the line
		if(vSearchIdx[0]){
			std::cout << "vSearchIdx[0]1: " << vSearchIdx[0] << std::endl;
			//if this point has been computed
			if (vLinePointStatus[vSearchIdx[0]] < 0){
				
			    //compute the moved location
				pcl::PointXY oNewAncher = MovingDistance(pLineCloud->points[vSearchIdx[0]],
					                                     m_pControlCloud->points[iQBoundIdx], fMoveDis);

				//compute the anchers
				pcl::PointXYZ oNew3DAncher;
				oNew3DAncher.x = oNewAncher.x;
				oNew3DAncher.y = oNewAncher.y;
				oNew3DAncher.z = pCloud->points[vSearchIdx[0]].z;

				//compute the 
				int iNewAncherIdx = oMap.AssignPointToMap(oNew3DAncher);
				std::cout <<" new.x: " << oNew3DAncher.x << std::endl;
				std::cout << "new.y: " << oNew3DAncher.y << std::endl;
				//if this new point is a ground point
				if (oMap.m_vReWardMap[iNewAncherIdx].travelable == 1){
					std::cout << "**3.0**" << std::endl;
					std::cout << "searched point: " << vSearchIdx[0] << std::endl;
					std::vector<int> vRadiuSIdx;
					std::vector<float> vRadiuSDis;
					
					//kdtree search
					oLineTree.radiusSearch(pLineCloud->points[vSearchIdx[0]], fMoveDis, vRadiuSIdx, vRadiuSDis);
					std::cout << vRadiuSIdx.size() << std::endl;
					std::cout << " old.x: " << pLineCloud->points[vRadiuSIdx[0]].x << std::endl;
					std::cout << " old.y: " << pLineCloud->points[vRadiuSIdx[0]].y << std::endl;
					for (int i = 0; i != vRadiuSIdx.size(); ++i)
						vLinePointStatus[vRadiuSIdx[i]] = iQSuccess;
					
					//add new data set
					vNewAncherPoints.push_back(oNew3DAncher);
					
			        iQSuccess++;
					
				}
				
			}//end if

		}//if point is
		
		iQBoundIdx++;
		
	}//while

	//make sure the first and last point is remained
	vLinePointStatus[0] = -1;
	vLinePointStatus[vLinePointStatus.size() - 1] = -1;

	//exchange
	std::vector<pcl::PointXYZ> vTempVec;
	//assigment
	int iLabel = -1;
	for (int i = 0; i != vLinePointStatus.size(); ++i) {
		//if changed
		if (vLinePointStatus[i] < 0) {
		
			vTempVec.push_back(pCloud->points[i]);
		
		}else if (vLinePointStatus[i] != iLabel) {
			std::cout << "got it!" << std::endl;
			vTempVec.push_back(vNewAncherPoints[vLinePointStatus[i]]);
			iLabel = vLinePointStatus[i];
		}
	
	}

	pCloud->points.clear();
	for (int i = 0; i != vTempVec.size(); ++i) {
	
		pCloud->points.push_back(vTempVec[i]);
	
	}
	
}

pcl::PointXY PathOptimization::MovingDistance(const pcl::PointXY & oLinePoint,
	                                           const pcl::PointXY & oObsPoint,
	                                                 const float & fMovingDis){
	
	//normalizetion
	//multiplied by the ratio
	float fKTime = fMovingDis / sqrt(pow(oLinePoint.x - oObsPoint.x, 2.0) +
		                             pow(oLinePoint.y - oObsPoint.y, 2.0));

    pcl::PointXY oMovedPoint;
	oMovedPoint.x = oLinePoint.x + fKTime * (oObsPoint.x - oLinePoint.x);
	oMovedPoint.y = oLinePoint.y + fKTime * (oObsPoint.y - oLinePoint.y);

	return oMovedPoint;

}