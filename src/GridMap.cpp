#include"GridMap.h"

//constructor
GridMap::GridMap(float f_fGridSize, 
	             float f_fMapTotalLength,
	             float f_fRadius,
	             float f_fMinThreshold)
	            :m_fMaskFlag(0.0),
	             m_iFilterNum(100){

	//set the size of map
	SetMapSize(f_fGridSize, f_fMapTotalLength);
	//set the min threshold of generating nodes
	SetSuppressionThrs(f_fRadius, f_fMinThreshold);
	//initialize map
	InitializeMap();
	
}

//destructor
GridMap::~GridMap() {

}

void GridMap::SetMapSize(float f_fGridSize, 
	                     float f_fMapTotalLength){
	
	m_fGridSize = f_fGridSize;
	m_fMapTotalLength = f_fMapTotalLength;

}

void GridMap::SetSuppressionThrs(float f_fRadius, 
	                             float f_fMinThreshold){
	
	m_fMinThreshold = f_fMinThreshold;
	m_fMinRadiusNum = f_fRadius;

}

void GridMap::InitializeMap(){

	m_iGridNum = int(floor(m_fMapTotalLength / m_fGridSize));

	//the corner of gird
	m_fMinGridCorner = -1.0 * (m_fMapTotalLength / 2.0);

	//scanning map
	m_vReWardMap.clear();
	m_vReWardMap.resize(m_iGridNum * m_iGridNum);

	//map that saves the normal vector
	m_vNormalVecMap.clear();
	m_vNormalVecMap.resize(m_iGridNum * m_iGridNum);
	
}

void GridMap::FiterGroundGrids(const std::vector<std::vector<int>> & vGridTravelIdx){

	for (int i = 0; i != m_vReWardMap.size(); ++i) {
	
		if (m_vReWardMap[i].iLabel == 2) {
			if (vGridTravelIdx[i].size() < m_iFilterNum) {
			
				m_vReWardMap[i].iLabel = 3;
			}
		}
			
	}

}


void GridMap::GenerateMap(std::vector<std::vector<int>> & vGridPointIdx){

	//map that saves the index of point
	vGridPointIdx.clear();
	vGridPointIdx.resize(m_iGridNum * m_iGridNum);

}

//compute the 
int GridMap::ComputeGridIdx(const int & iSX,
	                        const int & iSY){

	return iSY*m_iGridNum + iSX;

}


void GridMap::IntoXYSeries(int & iQuerySX,
	int & iQuerySY,
	const int & iQueryGridIdx) {

	iQuerySX = iQueryGridIdx % m_iGridNum;
	iQuerySY = (iQueryGridIdx - iQuerySX) / m_iGridNum;

}

//assign points to map
void GridMap::AssignPointsToMap(const PCLCloudXYZ & vCloud, 
	                                 std::vector<std::vector<int>> & vGridPointIdx,
	                                 unsigned int iLabel){
	
	//the first term is for the obstacle grid (gird label would be covered by other result) 
	if(iLabel == 1){

	     //// Compute the size of the slice structure
	     for (int i = 0; i != vCloud.points.size(); ++i) {
	
		      int iSX = int(floor((vCloud.points[i].x - m_fMinGridCorner) / m_fGridSize));
		      if (iSX < 0 || iSX >= m_iGridNum)
			      break;

		      int iSY = int(floor((vCloud.points[i].y - m_fMinGridCorner) / m_fGridSize));
		      if (iSY < 0 || iSY >= m_iGridNum)
			      break;

			  int iSXY = ComputeGridIdx(iSX, iSY);
		      vGridPointIdx[iSXY].push_back(i);
		
			  //label it into map
			  if (m_vReWardMap[iSXY].iLabel < 1)
				  m_vReWardMap[iSXY].iLabel = iLabel;

	    }//end for

	//the second one is for the travelable region (covers obstacle result) 
	}else if (iLabel == 2) {//end if
	
		//// Compute the size of the slice structure
		for (int i = 0; i != vCloud.points.size(); ++i) {

			int iSX = int(floor((vCloud.points[i].x - m_fMinGridCorner) / m_fGridSize));
			if (iSX < 0 || iSX >= m_iGridNum)
				break;

			int iSY = int(floor((vCloud.points[i].y - m_fMinGridCorner) / m_fGridSize));
			if (iSY < 0 || iSY >= m_iGridNum)
				break;

			int iSXY = ComputeGridIdx(iSX, iSY);
			vGridPointIdx[iSXY].push_back(i);

			//labels it into map
			if(m_vReWardMap[iSXY].iLabel < 2)
			   m_vReWardMap[iSXY].iLabel = iLabel;

		}//end for
    
	//belows are for the boundary (covers another result) 
	}else if (iLabel == 3) {//end else if
		//// Compute the size of the slice structure
		for (int i = 0; i != vCloud.points.size(); ++i) {

			int iSX = int(floor((vCloud.points[i].x - m_fMinGridCorner) / m_fGridSize));
			if (iSX < 0 || iSX >= m_iGridNum)
				break;

			int iSY = int(floor((vCloud.points[i].y - m_fMinGridCorner) / m_fGridSize));
			if (iSY < 0 || iSY >= m_iGridNum)
				break;

			int iSXY = ComputeGridIdx(iSX, iSY);
			vGridPointIdx[iSXY].push_back(i);

			//labels it into map
			if (m_vReWardMap[iSXY].iLabel < 3)
			m_vReWardMap[iSXY].iLabel = iLabel;

		}//end for

	}//end else if

}

int GridMap::AssignPointToMap(const pcl::PointXYZ & oQueryPoint) {

	//
	int iSX = int(floor((oQueryPoint.x - m_fMinGridCorner) / m_fGridSize));
	if (iSX < 0 || iSX >= m_iGridNum)
		return -1;

	//
	int iSY = int(floor((oQueryPoint.y - m_fMinGridCorner) / m_fGridSize));
	if (iSY < 0 || iSY >= m_iGridNum)
		return -1;	

	//
	return ComputeGridIdx(iSX, iSY);
		
}
//int GridMap::AssignPointToMap(const pcl::PointXYZ & oQueryPoint,
//	m_fMinGridCorner,
//	m_fGridSize) {
//
//	//
//	int iSX = int(floor((oQueryPoint.x - m_fMinGridCorner) / m_fGridSize));
//	if (iSX < 0 || iSX >= m_iGridNum)
//		return -1;
//
//	//
//	int iSY = int(floor((oQueryPoint.y - m_fMinGridCorner) / m_fGridSize));
//	if (iSY < 0 || iSY >= m_iGridNum)
//		return -1;
//
//	//
//	return ComputeGridIdx(iSX, iSY);
//
//}

//sampling point clouds
void GridMap::SamplingCloudUsingMap(int & numSampling) {


}

inline bool GridMap::JudgeRefreshMask(const float & fRadiusGridsNum) {
	//if they are the same (very very close in float type)
	if (fabs(m_fMaskFlag - fRadiusGridsNum) <= 1e-6)
		return false;//answer no to refresh mask
	else
		return true;//answer yes to refresh mask

}





std::vector<int> GridMap::SearchGrids(const int & iQueryGridIdx,
	                                  const float & fRadiusGridsNum){

	//define output
	std::vector<int> vNearbyGrids;
	//generate the mask first
	if(JudgeRefreshMask(fRadiusGridsNum))
	   GenerateSearchMask(fRadiusGridsNum);

	int iQuerySX;
	int iQuerySY;
	//get the index of query grid 
	IntoXYSeries(iQuerySX, iQuerySY,iQueryGridIdx);
	
	//if the query grid is on the edge of map
	//just return the empty value
	int iRadiusGridsNum = ceil(fRadiusGridsNum);
	if (iQuerySX < iRadiusGridsNum || iQuerySX >= m_iGridNum - iRadiusGridsNum)
		return vNearbyGrids;
	if (iQuerySY < iRadiusGridsNum || iQuerySY >= m_iGridNum - iRadiusGridsNum)
		return vNearbyGrids;

	//seach in given neighboring grids
	for (int i = 0; i != m_vSearchMask.size(); ++i) {
		
		//get nearby grid idx on x,y axis,respectively
		int iNearSX = iQuerySX + m_vSearchMask[i].ix;
		int iNearSY = iQuerySY + m_vSearchMask[i].iy;
		//compute the nearby grid
		vNearbyGrids.push_back(ComputeGridIdx(iNearSX, iNearSY));
	}

	//
    return vNearbyGrids;

}

void GridMap::GenerateSearchMask(const float & fRadiusGridsNum){

	//actual raduis grids number
	float fRealRGNum = fRadiusGridsNum + 0.5;
	float fMaskStartGrid = float(floor(-fRadiusGridsNum));
	float fMaskEndGrid = float(ceil(fRadiusGridsNum));

	m_vSearchMask.clear();
	//for x
	for (float i = fMaskStartGrid; i <= fMaskEndGrid; i = i + 1.0) {
		//for y
		for (float j = fMaskStartGrid; j <= fMaskEndGrid; j = j + 1.0) {
		//
			if (sqrt(pow(i, 2.0f) + pow(j, 2.0f)) <= fRealRGNum) {
				GridSearchMask oMaskMember;
				oMaskMember.ix = int(i);
				oMaskMember.iy = int(j);
				m_vSearchMask.push_back(oMaskMember);
			}//end if sqrt
		}//end j
	
	}//end i

}

//non-minimum suppression
std::vector<int> GridMap::NonMinimumSuppression(){

	//define candidate variables
	std::vector<int> vMinCandidates;
	std::vector<bool> vMinCandidStatus(m_vReWardMap.size(),true);

	//to each one
	for (int i = 0; i != m_vReWardMap.size(); ++i) {
		//if it is know and it is a ground grid
		if (m_vReWardMap[i].bKnownFlag && m_vReWardMap[i].iLabel == 2){
			//if it is a reachable grid
			if(m_vReWardMap[i].travelable == 1){
				//if it has not been computed
			    if (!m_vReWardMap[i].bMinFlag) {
				    //if it is small
				    if (m_vReWardMap[i].totalValue < m_fMinThreshold){
						
				        vMinCandidates.push_back(i);
			        }//end if m_vReWardMap[i].totalValue < m_fMinThreshold
			    }//end if !m_vReWardMap[i].bMinFlag
		    }//end if m_vReWardMap[i].travelable = 1
		}//end if m_vReWardMap[i].bKnownFlag && m_vReWardMap[i].iLabel == 2
	}//end for i
	
	//Traversal each candidate grid
	for (int i = 0; i != vMinCandidates.size(); ++i) {
		
		int iCurIdx = vMinCandidates[i];
		//if the candidate grid has not been removed
		if (vMinCandidStatus[iCurIdx]) {
			//find neighboring grid
			std::vector<int> vNeighborGrids = SearchGrids(iCurIdx, m_fMinRadiusNum);
			//search each neighboring grid
			for (int j = 0; j != vNeighborGrids.size(); ++j) {
				//the contrastive grid must be a travelable grid
				//this is because the non-ground grid alawys has a zero total value
				if(m_vReWardMap[vNeighborGrids[j]].travelable == 1){
					
				   //do not compare with itself(query grid)
				   if (iCurIdx != vNeighborGrids[j]) {
					   //compare the total value
					   if (m_vReWardMap[iCurIdx].totalValue <= m_vReWardMap[vNeighborGrids[j]].totalValue)
					       vMinCandidStatus[vNeighborGrids[j]] = false;
				       else 
					       vMinCandidStatus[iCurIdx] = false;
			       }//if != vNeighborGrids[j]
				}//end if(m_vReWardMap[iCurIdx].travelable == 1)
			}//end for j
		}//end if (vMinCandidates[vMinCandidates[i]])
			m_vReWardMap[iCurIdx].bMinFlag = true;
	}//end i != m_vReWardMap.size()
	
	//define output
	std::vector<int> vNodeIdx;
    //assignment
	for (int i = 0; i != vMinCandidates.size(); ++i){
		if (vMinCandidStatus[vMinCandidates[i]])
			vNodeIdx.push_back(vMinCandidates[i]);
	}

	return vNodeIdx;

}

/*************************************************
Function: RegionGrow
Description: this function is to find the reachable grid based on current robot location
Calls: none
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vNewScannedGrids - the neighboring grid of the current robot location
Output: the reachable label of grid map. The grid labelled as 1 is reachable grid 
Return: none
Others: point with label 2 is queried and changed one by one during implementing function
        point with label 0 will becomes point with label 2 after implementing function, this is to prepare next computing
*************************************************/
void GridMap::RegionGrow(const std::vector<int> & vNewScannedGrids){

    //status of grid in region grow
	//-1 indicates it is an unknown grid
	//0 indicates this grid is ground but not reachable now
    //1 indicates this grid is a travelable region grid
	//2 is the new scanned grids (input) without growing
	//3 indicates the grid has been computed
	//4 indicates this grid is a off groud grid (not reachable forever)
	
	for(int i=0;i!=vNewScannedGrids.size();++i){
		//if it is unknown (never be computed before)
		if(m_vReWardMap[vNewScannedGrids[i]].travelable == -1){
			//if it is a ground grid
			if(m_vReWardMap[vNewScannedGrids[i]].iLabel == 2)
		       m_vReWardMap[vNewScannedGrids[i]].travelable = 2;
			else
			   m_vReWardMap[vNewScannedGrids[i]].travelable = 4;//off-ground
		}
	}

    //check each input grid
	for(int i = 0;i!=vNewScannedGrids.size();++i){

		//current seed index
	    int iCurIdx;
		//inital flag as 3
		int bTravelableFlag = 3;
		
		//seeds
	    std::vector<int> vSeeds;  
		std::vector<int> vSeedHistory;
		//get one input grid as seed
		vSeeds.push_back(vNewScannedGrids[i]);

	    //if this one has not been grown
		if(m_vReWardMap[vNewScannedGrids[i]].travelable == 2){
            //growing
	        while(!vSeeds.empty()){
		         //choose a seed (last one)
		         iCurIdx = *(vSeeds.end()-1);
			
		         //delete the last one
		         vSeeds.pop_back();
				 //if this seeds has been calculated
				 if (m_vReWardMap[iCurIdx].travelable == 2) {
					 //record the history of seeds
					 vSeedHistory.push_back(iCurIdx);
					 //label this seed as being processed
					 m_vReWardMap[iCurIdx].travelable = 3;
					 //find the nearboring grids
					 std::vector<int> vNearGridIdx = SearchGrids(iCurIdx, 0.5);
					 
					 //check neighboring grids
					 for (int i = 0; i != vNearGridIdx.size(); ++i) {
						 //if the near grid is new input
						 if (m_vReWardMap[vNearGridIdx[i]].travelable == 2)
							 vSeeds.push_back(vNearGridIdx[i]);

						 //if the near grid is reachable so that the query ground grids must be also reachable
						 if (m_vReWardMap[vNearGridIdx[i]].travelable == 1)
							 bTravelableFlag = 2;
					 }//end for int i = 0;i!=vNearGridIdx.size();++i
				 }//end if m_vReWardMap[iCurIdx].travelable == 2
		    }//end while

		    //assignment
			for(int i=0;i!=vSeedHistory.size();++i){
			        m_vReWardMap[vSeedHistory[i]].travelable -= bTravelableFlag;
			}

	    }//end if m_vReWardMap[vNearGridIdx[i]].travelable == 2
	
	}//end for

	//search the unreachable grid
	//the reachable region is with respect to current query location
	//because an unreachable grid may be reachable if the robot moves close
	for (int i = 0; i != vNewScannedGrids.size(); ++i) {
	    //prepare for further growing
		if(m_vReWardMap[vNewScannedGrids[i]].travelable == 0)
		   m_vReWardMap[vNewScannedGrids[i]].travelable = 2;
	}

}