#include "Confidence.h"


/*************************************************
Function: Confidence
Description: constrcution function for Confidence class
Calls: SetSigmaValue
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: f_fSigma - the parameter sigma of GaussianKernel
Output: none
Return: none
Others: none
*************************************************/
Confidence::Confidence(float f_fSigma,
	                   float f_fGHPRParam,
	                  float f_fVisTermThr):
	                     m_fWeightDis(0.9),
                         m_fWeightVis(0.1){

	SetSigmaValue(f_fSigma);

	SetVisParas(f_fGHPRParam, f_fVisTermThr);

}


/*************************************************
Function: ~Confidence
Description: destrcution function for Confidence class
Calls: all member functions
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
Confidence::~Confidence(){


}

/*************************************************
Function: SetSigmaValue
Description: set value to the private data member m_fSigma
Calls: none
Called By: Confidence
Table Accessed: none
Table Updated: none
Input: f_fSigma - a given sigma value depends on the scanning region
Output: none
Return: none
Others: none
*************************************************/
void Confidence::SetSigmaValue(const float & f_fSigma) {

	m_fSigma = f_fSigma;

}

/*************************************************
Function: SetVisParas
Description: set value to the parameters related to visibility term
Calls: none
Called By: Confidence
Table Accessed: none
Table Updated: none
Input: f_fGHPRParam - a parameter of GHPR algorithm
      f_fVisTermThr - a threshold of visibility value
Output: m_fGHPRParam
        m_fVisTermThr
Return: none
Others: none
*************************************************/
void Confidence::SetVisParas(const float & f_fGHPRParam, 
	                         const float & f_fVisTermThr){

	m_fGHPRParam = f_fGHPRParam;

	m_fVisTermThr = f_fVisTermThr;

}


/*************************************************
Function: VectorInnerProduct
Description: This is an inner product operation of two vectors
Calls: none
Called By: DistanceTerm
Table Accessed: none
Table Updated: none
Input: oAVec - vector A
       oBVec - vector B
Output: the inner product value of vector A and vector B
Return: an inner product value
Others: none
*************************************************/
float Confidence::VectorInnerProduct(const pcl::PointXYZ & oAVec,
	                                 const pcl::PointXYZ & oBVec){

	//a*b = xa*xb + ya*yb + za*zb
	return oAVec.x * oBVec.x + oAVec.y * oBVec.y + oAVec.z * oBVec.z;

}

/*************************************************
Function: GaussianKernel
Description: This is a Gaussian Kernel Function to smooth the distance value between two points
Calls: Compute2Norm
Called By: DistanceTerm
Table Accessed: none
Table Updated: none
Input: oQueryPo - the query point (based point)
       oTargerPo - the target point 
	   sigma - the parameter that controls the affect radius of Gaussian Function, therefore the value
	   of sigma is normally set as half of searched radius 
Output: none
Return: none
Others: none
*************************************************/
inline float Confidence::GaussianKernel(const pcl::PointXYZ & oQueryPo,
	                                    const pcl::PointXYZ & oTargerPo, 
	                                                      float & sigma){

	// k(|| x - xc || ) = exp{ -|| x - xc || ^ 2 / (2 * sigma^2) }
	//or k(|| x - xc || ) = exp{ -|| x - xc || ^ 2 / (sigma^2) }

	//distance between two input vector
	float fNormSquare = Compute2Norm(oQueryPo, oTargerPo);
	
	      fNormSquare = pow(fNormSquare, 2.0f);
	//ouput equation result
	return exp(-1.0f * fNormSquare / pow(sigma,2.0f));
	
}

/*************************************************
Function: Compute2Norm
Description: compute the Euclidean distance between two points
Calls: none
Called By: GaussianKernel
Table Accessed: none
Table Updated: none
Input: oQueryPo - the query point (based point)
       oTargerPo - the target point 
Output: none
Return: none
Others: This function is the same with ComputeEuclideanDis, but it is an online one
*************************************************/
inline float Confidence::Compute2Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo){

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPo.x - oTargerPo.x, 2.0f)
		      + pow(oQueryPo.y - oTargerPo.y, 2.0f)
		      + pow(oQueryPo.z - oTargerPo.z, 2.0f));

}

/*************************************************
Function: Compute1Norm
Description: compute the Euclidean distance between two points
Calls: none
Called By: GaussianKernel
Table Accessed: none
Table Updated: none
Input: oQueryPo - the query point (based point)
       oTargerPo - the target point 
Output: the Euclidean distance value between two points
Return: a distance value 
Others: This function is the same with ComputeEuclideanDis, but it is an online one
*************************************************/
inline float Confidence::Compute1Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo){

	return sqrt(pow(oQueryPo.x - oTargerPo.x, 2.0f)
		      + pow(oQueryPo.y - oTargerPo.y, 2.0f)
	          + pow(oQueryPo.z - oTargerPo.z, 2.0f));

}

/*************************************************
Function: ComputeCenter
Description: compute the center point of a given point set with quired index
Calls: none
Called By: DistanceTerm
Table Accessed: none
Table Updated: none
Input: vCloud - a 3D point clouds
       vPointIdx - the point index vector indicates which point need to be computed 
Output: centerpoint center point(position) of the queried point set
Return: centerpoint
Others: none
*************************************************/
pcl::PointXYZ Confidence::ComputeCenter(const PCLCloudXYZ & vCloud,
	                            const std::vector<int> & vPointIdx){

	//define output
	pcl::PointXYZ oCenter;
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	//check whether the input has point 
	if(!vPointIdx.size())
		return oCenter;

	//calculate the average
	for (int i = 0; i != vPointIdx.size(); ++i) {

		oCenter.x = oCenter.x + vCloud.points[vPointIdx[i]].x;
		oCenter.y = oCenter.y + vCloud.points[vPointIdx[i]].y;
		oCenter.z = oCenter.z + vCloud.points[vPointIdx[i]].z;

	}

	oCenter.x = oCenter.x / float(vPointIdx.size());
	oCenter.y = oCenter.y / float(vPointIdx.size());
	oCenter.z = oCenter.z / float(vPointIdx.size());

	//output
	return oCenter;

}

/*************************************************
Function: ComputeCenter - Reload
Description:  compute the center point of a given point set 
Input: vCloud - a 3D point clouds
*************************************************/
pcl::PointXYZ  Confidence::ComputeCenter(const PCLCloudXYZ & vCloud){

	//define output
	pcl::PointXYZ oCenter;
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	//check whether the input has point 
	if (!vCloud.points.size())
		return oCenter;

	//calculate the average
	for (int i = 0; i != vCloud.points.size(); ++i) {
	
		oCenter.x = oCenter.x + vCloud.points[i].x;
		oCenter.y = oCenter.y + vCloud.points[i].y;
		oCenter.z = oCenter.z + vCloud.points[i].z;
	
	}

	oCenter.x = oCenter.x / float(vCloud.points.size());
	oCenter.y = oCenter.y / float(vCloud.points.size());
	oCenter.z = oCenter.z / float(vCloud.points.size());

	//output
	return oCenter;

}

/*************************************************
Function: ComputeEuclideanDis
Description: compute the Euclidean distance between two points
Calls: none
Called By: main function of project or other classes
Table Accessed: none
Table Updated: none
Input: oQueryPoint - the query point (based point)
       oTargetPoint - the target point 
Output: the distance value
Return: a distance value
Others: This function is the same with Compute1Norm, but it is a static one
*************************************************/
float Confidence::ComputeEuclideanDis(pcl::PointXYZ & oQueryPoint, pcl::PointXYZ & oTargetPoint) {

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPoint.x - oTargetPoint.x, 2.0f)
		      + pow(oQueryPoint.y - oTargetPoint.y, 2.0f)
		      + pow(oQueryPoint.z - oTargetPoint.z, 2.0f));

}

/*************************************************
Function: DistanceTerm
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
       GaussianKernel
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vReWardMap - the confidence map (grid map)
	   oRobotPoint - the location of the robot  
	   vNeighborGrids - the neighboring grids based on the input robot location
	   vTravelCloud - the travelable point clouds (the ground point clouds)
	   vGridTravelPsIdx - the index of point within each grid to total travelable point clouds  
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
void Confidence::DistanceTerm(std::vector<CofidenceValue> & vReWardMap,
	                                               const pcl::PointXYZ & oRobotPoint,
	                                         const std::vector<int> & vNeighborGrids,
	                                                const PCLCloudXYZ & vTravelCloud,
	                          const std::vector<std::vector<int>> & vGridTravelPsIdx){

	//**********Measurement item************
	//intermediate variables
	std::vector<float> vDisPartValue;///<distance weight part 
	vDisPartValue.resize(vGridTravelPsIdx.size(),0.0);

	std::vector<float> vCenterPartValue;///<center weight part
	vCenterPartValue.resize(vGridTravelPsIdx.size(),0.0);

	std::vector<pcl::PointXYZ> vCenterPoints;///<center points of each neighboring grids
	vCenterPoints.reserve(vNeighborGrids.size());
	pcl::PointXYZ oZeroCenter;
	oZeroCenter.x = 0.0;
	oZeroCenter.y = 0.0;
	oZeroCenter.z = 0.0;
	//center of grid is x=0,y=0,z=0 if this grid is empty
	for (int i = 0; i != vNeighborGrids.size(); ++i)
		vCenterPoints.push_back(oZeroCenter);

	//count how many neighboring grid has point 
	float fNonEmptyNum = 0.0;
	//define total center of whole neighborhood
	pcl::PointXYZ oTotalCenter;
	oTotalCenter.x = 0.0;
	oTotalCenter.y = 0.0;
	oTotalCenter.z = 0.0;
	//the grid center at which the robot is
	float fMaxDisValue = -FLT_MAX;
	pcl::PointXYZ oRobotGridCenter;
	oRobotGridCenter.x = 0.0;
	oRobotGridCenter.y = 0.0;
	oRobotGridCenter.z = 0.0;

	//**compute the distance part** 
	for (int i = 0; i != vNeighborGrids.size(); ++i) {
	
		//non-empty ground grid
		if (vReWardMap[vNeighborGrids[i]].iLabel == 2) {

			//compute the center of each neighboring grid
			vCenterPoints[i] = ComputeCenter(vTravelCloud, vGridTravelPsIdx[vNeighborGrids[i]]);

			//compute smooth distance using Gaussin Kernel based on the center point
			//the empty grid has zero value in this term
			vDisPartValue[i] = GaussianKernel(oRobotPoint, vCenterPoints[i], m_fSigma);

			//judge it is the robot grid
			if (fMaxDisValue < vDisPartValue[i]){
				fMaxDisValue = vDisPartValue[i];
			    oRobotGridCenter.x = vCenterPoints[i].x;
			    oRobotGridCenter.y = vCenterPoints[i].y;
			    oRobotGridCenter.z = vCenterPoints[i].z;
		    }

			//compute the center
			oTotalCenter.x = oTotalCenter.x + vCenterPoints[i].x;
			oTotalCenter.y = oTotalCenter.y + vCenterPoints[i].y;
			oTotalCenter.z = oTotalCenter.z + vCenterPoints[i].z;
			fNonEmptyNum = fNonEmptyNum + 1.0;

		}//end if (vGridTravelPsIdx[vNeighborGrids[i]].size())

	}//end i
	
	//**compute the center part**
	//compute the center point 
	oTotalCenter.x = oTotalCenter.x / fNonEmptyNum;
	oTotalCenter.y = oTotalCenter.y / fNonEmptyNum;
	oTotalCenter.z = oTotalCenter.z / fNonEmptyNum;
	oShowCenter.x = oTotalCenter.x;
	oShowCenter.y = oTotalCenter.y;
	oShowCenter.z = oTotalCenter.z;
	oShowRobot.x = oRobotGridCenter.x;
	oShowRobot.y = oRobotGridCenter.y;
	oShowRobot.z = oRobotGridCenter.z;
	
    //compute the offset vector from query point to the center point 
	pcl::PointXYZ oCenterOffVec;
	oCenterOffVec.x = oTotalCenter.x - oRobotGridCenter.x;
	oCenterOffVec.y = oTotalCenter.y - oRobotGridCenter.y;
	oCenterOffVec.z = oTotalCenter.z - oRobotGridCenter.z;
	
	//define a parameter of the center value to normalize the center part value (between -1 and 1)
	float fCenterNorPara = m_fSigma * m_fSigma;
	//for each non-empty neighboring grid
	for(int i=0;i!=vNeighborGrids.size();++i){
		//non-empty
		if (vGridTravelPsIdx[vNeighborGrids[i]].size()) {
		//compute the grid direction vector from query point to the grid center point  
		pcl::PointXYZ vGridVector;
		vGridVector.x = vCenterPoints[i].x - oRobotGridCenter.x;
		vGridVector.y = vCenterPoints[i].y - oRobotGridCenter.y;
		vGridVector.z = vCenterPoints[i].z - oRobotGridCenter.z;
		//
	    vCenterPartValue[i] = VectorInnerProduct(oCenterOffVec,vGridVector);
		vCenterPartValue[i] = 1.0 - vCenterPartValue[i] / fCenterNorPara;
		
		}//end if (vGridTravelPsIdx[vNeighborGrids[i]].size())

	}//end i 


	//** compute the total result** 
	//std::vector<float> vTotalDisValue;
	//for(int i=0;i!=vNeighborGrids.size();++i){
	//	vTotalDisValue.push_back(vCenterPartValue[i] * vDisPartValue[i]);
    //}

    //**********Incremental item************
	//fd(p) = max(fd(pi))  
	for(int i=0;i!=vNeighborGrids.size();++i){
		//get maximum value of distance term
		if(vReWardMap[vNeighborGrids[i]].disTermVal < vDisPartValue[i] * vCenterPartValue[i])
		   vReWardMap[vNeighborGrids[i]].disTermVal = vDisPartValue[i] * vCenterPartValue[i];
		//here is the case that only distance item works
		//if(vReWardMap[vNeighborGrids[i]].disTermVal < vDisPartValue[i])
		//   vReWardMap[vNeighborGrids[i]].disTermVal = vDisPartValue[i];
		
	}

}

/*************************************************
Function: QualityTerm
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
GaussianKernel
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vReWardMap - the confidence map (grid map)
oRobotPoint - the location of the robot
vNeighborGrids - the neighboring grids based on the input robot location
vTravelCloud - the travelable point clouds (the ground point clouds)
vGridTravelPsIdx - the index of point within each grid to total travelable point clouds
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
void Confidence::QualityTerm(std::vector<CofidenceValue> & vReWardMap,
	                                const pcl::PointXYZ & oRobotPoint,
	                          const std::vector<int> & vNeighborGrids,
	                                 const PCLCloudXYZ & vTravelCloud,
	           const std::vector<std::vector<int>> & vGridTravelPsIdx) {

	//define output
	std::vector<float> vDisRes;
	vDisRes.resize(vGridTravelPsIdx.size(), 0.0);

	//

}

/*************************************************
Function: OcclusionTerm
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
GaussianKernel
Called By: main function of project
Table Accessed: none
Table Updated: none
Input: vReWardMap - the confidence map (grid map)
oRobotPoint - the location of the robot
vNeighborGrids - the neighboring grids based on the input robot location
vTravelCloud - the travelable point clouds (the ground point clouds)
vGridTravelPsIdx - the index of point within each grid to total travelable point clouds
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
void Confidence::OcclusionTerm(std::vector<CofidenceValue> & vReWardMap,
	                            const std::vector<int> & vNeighborGrids,
				  const std::vector<pcl::PointXYZ> & vHistoryViewPoints,
	                                   const PCLCloudXYZ & vTravelCloud,
	             const std::vector<std::vector<int>> & vGridTravelPsIdx,
				                     const PCLCloudXYZ & vAllBoundCloud,
				  const std::vector<std::vector<int>> & vGridBoundPsIdx,
	                                 const PCLCloudXYZ & vObstacleCloud,
	                const std::vector<std::vector<int>> & vGridObsPsIdx){


	//point clouds to be seen
	PCLCloudXYZPtr pOccCloud(new PCLCloudXYZ);
	//point index to the travel grid
	std::vector<int> vTravelPointBelongIdx;

	//save the point that is in a reachable grid to compute a confidence map
	for (int i = 0; i != vNeighborGrids.size(); ++i) {

		int iOneGridIdx = vNeighborGrids[i];
		//if it is a ground grid
		if (vReWardMap[iOneGridIdx].iLabel == 2) {

			//record the ground point only
			for (int j = 0; j != vGridTravelPsIdx[iOneGridIdx].size(); ++j) {
				pOccCloud->points.push_back(vTravelCloud.points[vGridTravelPsIdx[iOneGridIdx][j]]);
				vTravelPointBelongIdx.push_back(i);
			}//end for j

		}//end if vReWardMap[vNeighborGrids[i]].iLabel == 2

	}//end for i

	//save the point that is in an unreachable grid
	for (int i = 0; i != vNeighborGrids.size(); ++i) {

		int iOneGridIdx = vNeighborGrids[i];

		if (vReWardMap[iOneGridIdx].iLabel != 2) {
			//record the ground point
			for (int j = 0; j != vGridTravelPsIdx[iOneGridIdx].size(); ++j)
				pOccCloud->points.push_back(vTravelCloud.points[vGridTravelPsIdx[iOneGridIdx][j]]);

			//record the boundary point
			for (int j = 0; j != vGridBoundPsIdx[iOneGridIdx].size(); ++j) 
				pOccCloud->points.push_back(vAllBoundCloud.points[vGridBoundPsIdx[iOneGridIdx][j]]);

			//record the obstacle point
			for (int j = 0; j != vGridObsPsIdx[iOneGridIdx].size(); ++j)
				pOccCloud->points.push_back(vObstacleCloud.points[vGridObsPsIdx[iOneGridIdx][j]]);

		}//end if vReWardMap[vNeighborGrids[i]].iLabel != 2

	}//end for i


	//std::vector<int> vVisableIdx = ComputeVisibility(*pOccCloud, oRobotPoint);
	GHPR oGHPRer(m_fGHPRParam);

	for (int i = 0; i != vHistoryViewPoints.size(); ++i) {
	
		//**********Measurement item************
		//intermediate variables
		std::vector<bool> vNearGridOccValue(vNeighborGrids.size(), false);///<distance weight part 

	    //compute the visibility based on the history of view points
		std::vector<int> vVisableIdx = oGHPRer.ComputeVisibility(*pOccCloud, vHistoryViewPoints[i]);
	
		//std::stringstream teststream;
		//teststream << RECORDNUM <<"_"<<i<< "Res.txt";
		//std::cout << teststream.str() <<" point size:  " << pOccCloud->points.size() << std::endl;
		//std::string testfilename;
		//teststream >> testfilename;
		//std::ofstream oRecordedFile;
		//oRecordedFile.open(testfilename.c_str(), std::ios::out | std::ios::app);
		//std::vector<int> vRes(pOccCloud->points.size(),0);
		//for (int i = 0; i != vVisableIdx.size(); ++i)
		//	vRes[vVisableIdx[i]] = 1;

		//for (int i=0;i!= pOccCloud->points.size();++i){
		////record the data in txt file for test
		//oRecordedFile << pOccCloud->points[i].x << " "
		//	          << pOccCloud->points[i].y << " "
		//	          << pOccCloud->points[i].z << " "
		//	          << vRes[i] << " "
		//	          << std::endl;
		//}
		//oRecordedFile << vHistoryViewPoints[i].x << " "
		//	<< vHistoryViewPoints[i].y << " "
		//	<< vHistoryViewPoints[i].z << " "
		//	<< 2 << " "
		//	<< std::endl;

		//visibility result assignment 
		for (int j = 0; j != vVisableIdx.size(); ++j){
		
		    //if it is in a travelable region 
			if (vVisableIdx[j] < vTravelPointBelongIdx.size()) {
				vNearGridOccValue[vTravelPointBelongIdx[vVisableIdx[j]]] = true;
			}
		
		}
	
	    //**********Incremental item************
	    //fv(p) = fv(n)  
	    for (int i = 0; i != vNeighborGrids.size(); ++i){
		     //get maximum value of distance term
			 if(vNearGridOccValue[i])
			    vReWardMap[vNeighborGrids[i]].visibility += 1.0;

			 //if the visibility value is up to upper bound
			 if (vReWardMap[vNeighborGrids[i]].visibility > m_fVisTermThr)
				 vReWardMap[vNeighborGrids[i]].visibility = m_fVisTermThr;
	    }

	}//end for (int i = 0; i != vHistoryViewPoints.size(); ++i) 

}

/*************************************************
Function: BoundaryTerm
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
       GaussianKernel
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vReWardMap - the confidence map (grid map)
	   oRobotPoint - the location of the robot  
	   vNeighborGrids - the neighboring grids based on the input robot location
	   vTravelCloud - the travelable point clouds (the ground point clouds)
	   vGridTravelPsIdx - the index of point within each grid to total travelable point clouds  
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
//std::vector<float> Confidence::BoundaryTerm(PCLCloudXYZ & vTravelCloud, PCLCloudXYZ & vBoundCloud, pcl::PointXYZ & oRobotPoint){
//	
//	//new output
//	std::vector<float> vBoundRes;
//	if (!vBoundCloud.points.size()) {
//		vBoundRes.resize(vTravelCloud.points.size(), ROBOT_AFFECTDIS);
//		return vBoundRes;
//	}
//	std::cout << "bound points: " << vBoundCloud.points.size() << std::endl;
//	//preparation
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pBoundKDCloud(new pcl::PointCloud<pcl::PointXYZ>);
//	for (int i = 0; i != vBoundCloud.points.size(); ++i)
//		pBoundKDCloud->points.push_back(vBoundCloud.points[i]);
//
//	//new a kdtree for query point clouds
//	pcl::KdTreeFLANN<pcl::PointXYZ> oBoundKDTree;
//	oBoundKDTree.setInputCloud(pBoundKDCloud);
//
//	//
//	for (int i = 0; i < vTravelCloud.points.size(); ++i) {
//
//		//
//		std::vector<int> vSearchedIdx;
//		std::vector<float> vSearchedDis;
//		//
//		oBoundKDTree.nearestKSearch(vTravelCloud.points[i], 1, vSearchedIdx, vSearchedDis);
//		//
//		//if (vSearchedDis[0] > 5.0)
//		//	vSearchedDis[0] = 5.0;
//
//		vBoundRes.push_back(vSearchedDis[0]);
//
//	}//end i
//
//	return vBoundRes;
//
//}


/*************************************************
Function: FrontierTerm
Description: the function is to compute the frontier feature, which is popular in roboticmethod
Calls: none
       none
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vReWardMap - the confidence map (grid map)
	   iQueryGrid - the grid at which the robot right now is
	   vNeighborGrids - the neighboring grids index based on the robot grid
Output: change the confidence value about frontier part
Return: none
Others: none
*************************************************/
//void Confidence::FrontierTerm(std::vector<CofidenceValue> & vReWardMap, 
//	                                            const int & iQueryGrid,
//	                           const std::vector<int> & vNeighborGrids){
//
//	//variables
//	float fBoundaryRes = 0.0;
//	int iUnkownCount = 0;
//
//	//if it is a ground region
//	if (vReWardMap[iQueryGrid].iLabel == 2) {
//
//		for (int k = 0; k != vNeighborGrids.size(); ++k) {
//			//count its neighboring unknown grids
//			if (!vReWardMap[vNeighborGrids[k]].bKnownFlag) {
//				iUnkownCount++;
//			}
//
//		}//end for k
//
//	}//end if vReWardMap
//
//	//it has a high value if it is far away from the boundary 
//	if (!iUnkownCount)
//		fBoundaryRes = 1.0;
//
//	vReWardMap[iQueryGrid].boundary = fBoundaryRes;
//
//}



/*************************************************
Function: ComputeTotalCoffidence
Description: the function is to compute the distance feature to the confidence value
Calls: ComputeCenter
       GaussianKernel
Called By: main function of project 
Table Accessed: none
Table Updated: none
Input: vReWardMap - the confidence map (grid map)
	   oRobotPoint - the location of the robot  
	   vNeighborGrids - the neighboring grids based on the input robot location
	   vTravelCloud - the travelable point clouds (the ground point clouds)
	   vGridTravelPsIdx - the index of point within each grid to total travelable point clouds  
Output: the distance term value of each neighboring grid
Return: a vector saves distance value of each neighboring grid
Others: none
*************************************************/
void Confidence::ComputeTotalCoffidence(std::vector<CofidenceValue> & vReWardMap, 
	                                     const std::vector<int> & vNeighborGrids){

	//to each searched grid
	for (int i = 0; i != vNeighborGrids.size(); ++i) {
		//define query index
		int iQueryIdx = vNeighborGrids[i];
		//if this region is a ground point 
		if (vReWardMap[iQueryIdx].iLabel == 2) {
			//the grid is known
			if (vReWardMap[iQueryIdx].bKnownFlag)
			    vReWardMap[iQueryIdx].totalValue = m_fWeightDis * vReWardMap[iQueryIdx].disTermVal 
				                                 + m_fWeightVis * vReWardMap[iQueryIdx].visibility;
			
		}//end if vReWardMap[iQueryIdx].iLabel == 2
	}//end for i = 0

}




/*************************************************
Function: Normalization
Description: normalize the input feature vector
Calls: none
Called By: major function
Table Accessed: none
Table Updated: none
Input: vFeatures - a feature (feature value of each grid or feature value of each point)
Output: change the feature value and limit it between 0 and 1
Return: none
Others: 0 <= vFeatures[i] <= 1
*************************************************/
void Confidence::Normalization(std::vector<float> & vFeatures){

	//maximam and minimam
	float fMaxValue = -FLT_MAX;
	float fMinValue = FLT_MAX;

	//find the maximun and minmum value
	for (int i = 0; i != vFeatures.size(); ++i) {
	
		if (vFeatures[i] > fMaxValue)
			fMaxValue = vFeatures[i];
		if (vFeatures[i] < fMinValue)
			fMinValue = vFeatures[i];
	}

	//Normalization
	for (int i = 0; i != vFeatures.size(); ++i) 
		vFeatures[i] = (vFeatures[i] - fMinValue) / (fMaxValue - fMinValue);
	
}

