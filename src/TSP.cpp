#include "TSP.h"

TSP::TSP(){



}


TSP::~TSP(){


}

void TSP::GetCurrentLocation(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	                         const std::vector<std::vector<int>> & vGridPointIdx,
	                         const int & iRobot){

	//get the grid index of current robot location 
	m_iCurrentNodeIdx = iRobot;
	m_oCurrentCenter = ComputeCentersPosition(pCloud, vGridPointIdx, iRobot);
}

void TSP::GetCurNodesIdx(int f_iRbtPstion) {

	//get the grid at which the robot is
	m_iCurrentNodeIdx = f_iRbtPstion;

}


pcl::PointXYZ TSP::ComputeCentersPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	const std::vector<std::vector<int>> & vGridPointIdx,
	const int & iQueryIdx) {

	//define output
	pcl::PointXYZ oCenter;
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	int fAllNumber = vGridPointIdx[iQueryIdx].size();
	//find each points
	for (int j = 0; j != fAllNumber; ++j) {

		oCenter.x += pCloud->points[vGridPointIdx[iQueryIdx][j]].x;
		oCenter.y += pCloud->points[vGridPointIdx[iQueryIdx][j]].y;
		oCenter.z += pCloud->points[vGridPointIdx[iQueryIdx][j]].z;

	}

	//averaging
	oCenter.x = oCenter.x / float(fAllNumber);
	oCenter.y = oCenter.y / float(fAllNumber);
	oCenter.z = oCenter.z / float(fAllNumber);

	return oCenter;

}
//reload to output a center position with a successed flag
bool TSP::ComputeCentersPosition(pcl::PointXYZ oCenter,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	const std::vector<std::vector<int>> & vGridPointIdx,
	const int & iQueryIdx) {

	//define output
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	int fAllNumber = vGridPointIdx[iQueryIdx].size();
	//find each points
	for (int j = 0; j != fAllNumber; ++j) {

		oCenter.x += pCloud->points[vGridPointIdx[iQueryIdx][j]].x;
		oCenter.y += pCloud->points[vGridPointIdx[iQueryIdx][j]].y;
		oCenter.z += pCloud->points[vGridPointIdx[iQueryIdx][j]].z;

	}

	if (!fAllNumber)
		return false;

	//averaging
	oCenter.x = oCenter.x / float(fAllNumber);
	oCenter.y = oCenter.y / float(fAllNumber);
	oCenter.z = oCenter.z / float(fAllNumber);

	return true;

}


float TSP::ComputeEuclideanDis(const pcl::PointXYZ & oQueryPoint,
	                           const pcl::PointXYZ & oTargetPoint) {

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPoint.x - oTargetPoint.x, 2.0f)
		+ pow(oQueryPoint.y - oTargetPoint.y, 2.0f)
		+ pow(oQueryPoint.z - oTargetPoint.z, 2.0f));

}



void TSP::GetNewNode(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	                 const std::vector<std::vector<int>> & vGridPointIdx,
	                 const int & vNewNodeIdxs,
					 float fSuppressionR){

	float fMinDis = FLT_MAX;

	//to each node
	pcl::PointXYZ oCenterPoint
			= ComputeCentersPosition(pCloud, vGridPointIdx, vNewNodeIdxs);

	//find wether the input new node is near a unvisited node
	for(int i=0;i!=m_vUnVisitCenters.size();++i){
		//compute the distance
	    float fDis = ComputeEuclideanDis(oCenterPoint,m_vUnVisitCenters[i]);
		//record the shortest distance
		if(fDis < fMinDis)
		   fMinDis = fDis;
	}

	if(fMinDis > fSuppressionR){
	   //get the grid idx of new node
	   m_vUnVisitNodeIdx.push_back(vNewNodeIdxs);
	   //ger the center position of new node
	   m_vUnVisitCenters.push_back(oCenterPoint);
	}

}



void TSP::GetNewNodes(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	                  const std::vector<std::vector<int>> & vGridPointIdx,
	                  const std::vector<int> & vNewNodeIdxs,
					  float fSuppressionR){

    if(m_vUnVisitCenters.size()){

		 //new a point cloud to save thr unvisited point
         pcl::PointCloud<pcl::PointXYZ>::Ptr pUnVisitCloud(new pcl::PointCloud<pcl::PointXYZ>);
	     for (int i = 0; i != m_vUnVisitCenters.size(); ++i)
		      pUnVisitCloud->points.push_back(m_vUnVisitCenters[i]);

	     //new a kdtree for unvisited point clouds
	     pcl::KdTreeFLANN<pcl::PointXYZ> oUnVisitKDTree;
	     oUnVisitKDTree.setInputCloud(pUnVisitCloud);

	     //to each new node
	     for (int i = 0; i != vNewNodeIdxs.size(); ++i) {
			
			 //compute the center location of new grid 
			 pcl::PointXYZ oCenterPoint 
			             = ComputeCentersPosition(pCloud, vGridPointIdx,vNewNodeIdxs[i]);
		     //set the searched vector
		     std::vector<int> vSearchedIdx;
		     std::vector<float> vSearchedDis;
		     //find the nearest unvisited node
		     oUnVisitKDTree.nearestKSearch(oCenterPoint, 1, vSearchedIdx, vSearchedDis);

			 //if they are far away (the distance farther than threshold)
			 if(vSearchedDis[0] > fSuppressionR){
		        //get the grid idx of new node
		        m_vUnVisitNodeIdx.push_back(vNewNodeIdxs[i]);
		        //ger the center position of new node
		        m_vUnVisitCenters.push_back(oCenterPoint);
			 }//end if

		 }//end for int i = 0; i != vNewNodeIdxs.size(); ++i)

	}else{//there are not unvisited nodes
	//directly save input nodes
		//to each node
	    for (int i = 0; i != vNewNodeIdxs.size(); ++i) {
	
		    pcl::PointXYZ oCenterPoint 
		              	= ComputeCentersPosition(pCloud, vGridPointIdx,vNewNodeIdxs[i]);

		    //get the grid idx of new node
		    m_vUnVisitNodeIdx.push_back(vNewNodeIdxs[i]);
		    //get the center position of new node
		    m_vUnVisitCenters.push_back(oCenterPoint);
	
	    }//end for int i = 0; i != vNewNodeIdxs.size()

	}//end else

}



float TSP::CostFunction(const std::vector<CofidenceValue> & vReWardMap,
	      const int & vQueryIdx,
	      const pcl::PointXYZ & oQueryCenter,
	      const int & vTargetIdx,
          const pcl::PointXYZ & oTargetCenter){

	//define 
	float fFinalCost = FLT_MAX;

	//compute the final cost based on the cost and reward
	fFinalCost = ComputeEuclideanDis(oQueryCenter, oTargetCenter);

	//
	return fFinalCost;

}


bool TSP::GTR(const std::vector<CofidenceValue> & vReWardMap) {

	//it is true if the unvisited nodes has not been selected 
	std::vector<bool> vUnVisitedStatus(m_vUnVisitNodeIdx.size(), true);

	//the current robot position is the first query index
	int iQueryIdx = m_iCurrentNodeIdx;
	pcl::PointXYZ oQueryCenter = m_oCurrentCenter;
	int iTargetIdx;
	pcl::PointXYZ oTargetCenter;

	//new plan of nodes
	std::vector<int> vNewPlan;
	std::vector<pcl::PointXYZ> vNewPlanCenters;

	//if the robot successfully reached target node
	if (m_iCurrentNodeIdx == m_vUnVisitNodeIdx[0])
		m_vVisitedNodeIdx.push_back(m_vUnVisitNodeIdx[0]);
	else
		m_vVisitedNodeIdx.push_back(m_iCurrentNodeIdx);

	//refresh the unvisited nodes
	vUnVisitedStatus[0] = false;

	//check whether all are completed
	int iRemainNum = 0;
	for (int i = 0; i != vUnVisitedStatus.size();++i) {
		if (vUnVisitedStatus[i])
			iRemainNum++;
    }

	//define output
	//if there are not any new nodes shoule be invoked
	if (!iRemainNum){
		m_vUnVisitNodeIdx.clear();
		m_vUnVisitCenters.clear();
		return true;
    }

	//loop of each query point
	while(iRemainNum){
	
		float fMinCost = FLT_MAX;
	    int iMinIdx = 0;
		//to each unselected point
	    for (int i = 0; i != m_vUnVisitNodeIdx.size(); ++i) {

			if(vUnVisitedStatus[i]){
			   //compute the real cost using the given cost function
			   iTargetIdx = m_vUnVisitNodeIdx[i];
			   oTargetCenter = m_vUnVisitCenters[i];

			   //cost function which considers the reward and cost of node
		       float fPairCost = CostFunction(vReWardMap,iQueryIdx,oQueryCenter,
				                                     iTargetIdx,oTargetCenter);
			   
		       //find the shorest route
		       if (fPairCost < fMinCost) {
			       fMinCost = fPairCost;
			       iMinIdx = i;
		       }

			}//end if(vUnVisitedStatus[i])

	    }//end for i != vUnSelectedNodes.size()

	    //find the real index in unvisited nodes
        iQueryIdx  = m_vUnVisitNodeIdx[iMinIdx];
		oQueryCenter.x = m_vUnVisitCenters[iMinIdx].x;
		oQueryCenter.y = m_vUnVisitCenters[iMinIdx].y;
		oQueryCenter.z = m_vUnVisitCenters[iMinIdx].z;

	    //save the selected one
	    vNewPlan.push_back(iQueryIdx);
	    vNewPlanCenters.push_back(oQueryCenter);

		//label the selected node in unvisited nodes
	    vUnVisitedStatus[iMinIdx] = false;

		iRemainNum--;

    }//end while

	//clear old unvisted node
	m_vUnVisitNodeIdx.clear();
	m_vUnVisitCenters.clear();
	m_vUnVisitNodeIdx.resize(vNewPlan.size());
	m_vUnVisitCenters.resize(vNewPlanCenters.size());
	//save new unvisited node
	//the traversing order is based on the serial number
	
	for (int i = 0; i != vNewPlan.size(); ++i){
		m_vUnVisitNodeIdx[i] = vNewPlan[i];
		m_vUnVisitCenters[i] = vNewPlanCenters[i];
	}

	return false;

}



void TSP::OutputVisitedNodes(std::vector<int> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.resize(m_vVisitedNodeIdx.size());

	//assignment
	for (int i = 0; i != m_vVisitedNodeIdx.size();++i){
		vOutputNodes.push_back(m_vVisitedNodeIdx[i]);
	}

}

int TSP::OutputVisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	const std::vector<std::vector<int>> & vGridPointIdx){

	vOutputNodes.clear();
	vOutputNodes.reserve(m_vVisitedNodeIdx.size());

	//assignment with center value of grid
	for (int i = 0; i != m_vVisitedNodeIdx.size(); ++i) {
		
		pcl::PointXYZ oCenter = 
			ComputeCentersPosition(pCloud, vGridPointIdx, m_vVisitedNodeIdx[i]);

		vOutputNodes.push_back(oCenter);

	}

	return m_vVisitedNodeIdx.size();

}

void TSP::OutputUnVisitedNodes(std::vector<int> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.resize(m_vUnVisitNodeIdx.size());

	//assignment
	for (int i = 0; i != m_vUnVisitNodeIdx.size();++i){
		vOutputNodes.push_back(m_vUnVisitNodeIdx[i]);
	}

}


void TSP::OutputUnVisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.reserve(m_vUnVisitCenters.size());

	//assignment with center value of grid
	for (int i = 0; i != m_vUnVisitCenters.size(); ++i) {		
		vOutputNodes.push_back(m_vUnVisitCenters[i]);
	}

}