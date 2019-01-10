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
		pcl::PointXYZ oControlCenter;
		//if this grid has points(because the query gird only has boundary points sometimes)
		//and our input is the obstacle points

		if (TSP::ComputeCentersPosition(oControlCenter, pCloud, vGridPointIdx, m_vControls[i].idx)){
			//only record x,y
			pcl::PointXY oPoint;
			oPoint.x = oControlCenter.x;
			oPoint.y = oControlCenter.y;
			
			m_pControlCloud->points.push_back(oPoint);
		}//end if

	}//end for

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

	//if the distance is too short
	if (pLineCloud->points.size() < iMinPathLength)
		return;
	//construct a kdtree
    pcl::KdTreeFLANN<pcl::PointXY> oLineTree;
	oLineTree.setInputCloud(pLineCloud);

	//if can not constructed a tree
	if (!m_pControlCloud->points.size())
		return;//return if nothing input
	//if control points is less
	if (m_pControlCloud->points.size() < iMaxSearchTime)
		iMaxSearchTime = m_pControlCloud->points.size();

	int iQBoundIdx = 0;
	int iQSuccess = 0;//The successed seed
	
	//compute the index
	while (iQBoundIdx < iMaxSearchTime && iQSuccess < 3) {

		std::vector<int> vSearchIdx;
		std::vector<float> vSearchDis;
		//search distance of center of sign based on max distance to center

		oLineTree.nearestKSearch(m_pControlCloud->points[iQBoundIdx], 1, vSearchIdx, vSearchDis);

		//if the searched point is first point, which indicates the searched point is behind the line
		if(vSearchIdx[0] != 0 && vSearchIdx[0] != pLineCloud->points.size() - 1){
			
			//if this point has been computed
			if (vLinePointStatus[vSearchIdx[0]] < 0){

				//if this searched is far away from boundary
				int iMovingIdx = oMap.AssignPointToMap(pCloud->points[vSearchIdx[0]]);

				int iRadius = ceil(fMoveDis / float(oMap.m_fGridSize));
				std::vector<int> vMovingNearGrids = oMap.SearchGrids(iMovingIdx, iRadius);
				//a flag indicates that moving this point is safe
				bool bNonNearBoundFlag = true;
				for (int i = 0; i != vMovingNearGrids.size(); ++i) {
					if (oMap.m_vReWardMap[vMovingNearGrids[i]].iLabel == 1 ||
						oMap.m_vReWardMap[vMovingNearGrids[i]].iLabel == 3) {
						bNonNearBoundFlag = false;
					}
				}

				//if it is far away from boundary
				if (bNonNearBoundFlag) {
					
					//compute the moved location
					pcl::PointXY oNewAncher = MovingDistance(pLineCloud->points[vSearchIdx[0]],
						m_pControlCloud->points[iQBoundIdx], fMoveDis);

					//compute the anchers
					pcl::PointXYZ oNew3DAncher;
					oNew3DAncher.x = oNewAncher.x;
					oNew3DAncher.y = oNewAncher.y;
					oNew3DAncher.z = pCloud->points[vSearchIdx[0]].z;
					

					//compute the 
					//int iNewAncherIdx = oMap.AssignPointToMap(oNew3DAncher);
					std::vector<int> vRadiuSIdx;
					std::vector<float> vRadiuSDis;

					//kdtree search
					oLineTree.radiusSearch(pLineCloud->points[vSearchIdx[0]], fMoveDis, vRadiuSIdx, vRadiuSDis);

					for (int i = 0; i != vRadiuSIdx.size(); ++i){

						vLinePointStatus[vRadiuSIdx[i]] = iQSuccess;
						
                    }
					//add new data set
					vNewAncherPoints.push_back(oNew3DAncher);

					iQSuccess++;

				}
				
			}//end if

		}//if point is
		
		iQBoundIdx++;
		
	}//while

	std::cout << "Control Point Success Times:  " << iQSuccess << std::endl;

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