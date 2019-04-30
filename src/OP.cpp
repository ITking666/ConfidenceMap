#include "OP.h"

//5 is a placeholder
//it will be changed when using ObjectiveMatrix
OP::OP():BBSolver(5),
         m_fWideThr(0.5){



}


OP::~OP(){


}

void OP::GetCurrentLocation(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	                         const std::vector<std::vector<int>> & vGridPointIdx,
	                         const int & iRobot){

	//get the grid index of current robot location 
	m_iCurrentNodeIdx = iRobot;
	m_oCurrentCenter = ComputeCentersPosition(pCloud, vGridPointIdx, iRobot);
}

void OP::GetCurNodesIdx(int f_iRbtPstion) {

	//get the grid at which the robot is
	m_iCurrentNodeIdx = f_iRbtPstion;

}


pcl::PointXYZ OP::ComputeCentersPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
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
bool OP::ComputeCentersPosition(pcl::PointXYZ & oCenter,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
	const std::vector<std::vector<int>> & vGridPointIdx,
	const int & iQueryIdx) {

	//define output
	oCenter.x = 0.0;
	oCenter.y = 0.0;
	oCenter.z = 0.0;

	int fAllNumber = vGridPointIdx[iQueryIdx].size();

	if (!fAllNumber)
		return false;

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

	return true;

}


float OP::ComputeEuclideanDis(const pcl::PointXYZ & oQueryPoint,
	                           const pcl::PointXYZ & oTargetPoint) {

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPoint.x - oTargetPoint.x, 2.0f)
		+ pow(oQueryPoint.y - oTargetPoint.y, 2.0f)
		+ pow(oQueryPoint.z - oTargetPoint.z, 2.0f));

}



void OP::GetNewNode(const std::vector<CofidenceValue> & vReWardMap,
	                 const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
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

	   if (vReWardMap[vNewNodeIdxs].boundary > m_fWideThr)
		   m_vUnVisitedWide.push_back(true);
	   else
		   m_vUnVisitedWide.push_back(false);
	}

}



void OP::GetNewNodes(const std::vector<CofidenceValue> & vReWardMap,
	                 const pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud,
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
			 if(sqrt(vSearchedDis[0]) > fSuppressionR){
		        //get the grid idx of new node
		        m_vUnVisitNodeIdx.push_back(vNewNodeIdxs[i]);
		        //ger the center position of new node
		        m_vUnVisitCenters.push_back(oCenterPoint);

				if (vReWardMap[vNewNodeIdxs[i]].boundary > m_fWideThr)
					m_vUnVisitedWide.push_back(true);
				else
					m_vUnVisitedWide.push_back(false);
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

			if (vReWardMap[vNewNodeIdxs[i]].boundary > m_fWideThr)
				m_vUnVisitedWide.push_back(true);
			else
				m_vUnVisitedWide.push_back(false);
	
	    }//end for int i = 0; i != vNewNodeIdxs.size()

	}//end else

}

//refresh the nodes
bool OP::RefreshNodes(std::vector<CofidenceValue> & vReWardMap,
	                                       float fMinThreshold){

	//a status indicating whether the node is removed or not
	std::vector<bool> vStatus(m_vUnVisitNodeIdx.size(),true);
	for (int i = 0; i != m_vUnVisitNodeIdx.size(); ++i) {
		if(vReWardMap[m_vUnVisitNodeIdx[i]].totalValue > fMinThreshold)
			vStatus[i] = false;
	}

	//remain the current goal
	//the current goal will be removed in node strategy function
	if(m_vUnVisitNodeIdx.size())
		vStatus[0] = true;

	//new plan of nodes
	std::vector<int> vNewPlan;
	std::vector<pcl::PointXYZ> vNewPlanCenters;
	std::vector<bool> vNewPlanWides;
	for (int i = 0; i != vStatus.size(); ++i) {
		if (vStatus[i]){
			//save the selected one
			vNewPlan.push_back(m_vUnVisitNodeIdx[i]);
			vNewPlanCenters.push_back(m_vUnVisitCenters[i]);
			vNewPlanWides.push_back(m_vUnVisitedWide[i]);
		}
	}

	//clear old unvisted node
	m_vUnVisitNodeIdx.clear();
	m_vUnVisitCenters.clear();
	m_vUnVisitedWide.clear();
	m_vUnVisitNodeIdx.resize(vNewPlan.size());
	m_vUnVisitCenters.resize(vNewPlanCenters.size());
	m_vUnVisitedWide.resize(vNewPlanWides.size());
	//save new unvisited node
	//the traversing order is based on the serial number
	for (int i = 0; i != vNewPlan.size(); ++i) {
		m_vUnVisitNodeIdx[i] = vNewPlan[i];
		m_vUnVisitCenters[i] = vNewPlanCenters[i];
		m_vUnVisitedWide[i] = vNewPlanWides[i];
	}


	//wide node number
	int iNoWideNum = 0;
	//check whether there is still node in wide area
	//start from the next view point
	for (int i = 1; i < m_vUnVisitedWide.size(); ++i) {
		//if the node area is wide and it still need to be explored
		if (!m_vUnVisitedWide[i])
			iNoWideNum++;
	}

	//if there is still wide area
	std::cout << "remain nowide node num: " << iNoWideNum << std::endl;
	if (iNoWideNum)
		return false;
	else
		return true;

}


float OP::CostFunction(const std::vector<CofidenceValue> & vReWardMap,
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


float OP::ObjectiveFunction(const std::vector<CofidenceValue> & vReWardMap,
	                    const int & vQueryIdx,
	                    const pcl::PointXYZ & oQueryCenter,
	                    const int & vTargetIdx,
	                    const pcl::PointXYZ & oTargetCenter) {

	//define 
	//compute the final cost based on the cost and reward
	float fFinalCost = ComputeEuclideanDis(oQueryCenter, oTargetCenter);

	float fFinalReward = 1.0 - vReWardMap[vTargetIdx].totalValue;
	
	float fObjectVal = FLT_MAX;
	if (fFinalReward > 0.0)
		fObjectVal = fFinalCost / fFinalReward;

	//return result
	return fObjectVal;

}

bool OP::GTR(const std::vector<CofidenceValue> & vReWardMap) {

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
	std::vector<bool> vNewPlanWides;

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
		vNewPlanWides.push_back(m_vUnVisitedWide[iMinIdx]);

		//label the selected node in unvisited nodes
	    vUnVisitedStatus[iMinIdx] = false;

		iRemainNum--;

    }//end while

	//clear old unvisted node
	m_vUnVisitNodeIdx.clear();
	m_vUnVisitCenters.clear();
	m_vUnVisitedWide.clear();
	m_vUnVisitNodeIdx.resize(vNewPlan.size());
	m_vUnVisitCenters.resize(vNewPlanCenters.size());
	m_vUnVisitedWide.resize(vNewPlanWides.size());
	//save new unvisited node
	//the traversing order is based on the serial number
	
	for (int i = 0; i != vNewPlan.size(); ++i){
		m_vUnVisitNodeIdx[i] = vNewPlan[i];
		m_vUnVisitCenters[i] = vNewPlanCenters[i];
		m_vUnVisitedWide[i] = vNewPlanWides[i];
	}

	return false;

}


bool OP::BranchBoundMethod(const std::vector<CofidenceValue> & vReWardMap) {

	//the current robot position is the first query index
	int iQueryIdx = m_iCurrentNodeIdx;
	std::cout << "now the robot is at " << m_iCurrentNodeIdx << std::endl;
	pcl::PointXYZ oQueryCenter = m_oCurrentCenter;

	//if the robot successfully reached target node
	if (m_iCurrentNodeIdx == m_vUnVisitNodeIdx[0]){
		m_vVisitedNodeIdx.push_back(m_vUnVisitNodeIdx[0]);
	}else {
		m_vVisitedNodeIdx.push_back(m_iCurrentNodeIdx);
		m_vUnVisitNodeIdx[0] = m_iCurrentNodeIdx;
		m_vUnVisitCenters[0] = oQueryCenter;
	}
	

	//define output
	//if there are not any new nodes shoule be invoked
	if (m_vUnVisitNodeIdx.size() <= 1) {
		m_vUnVisitNodeIdx.clear();
		m_vUnVisitCenters.clear();
		return true;
	}

	//effective matrix
	std::vector<std::vector<float>> vEffectMatrix;
	std::vector<float> vArray(m_vUnVisitNodeIdx.size(), 0.0);
	for (int i = 0; i != m_vUnVisitNodeIdx.size(); ++i) {
		vEffectMatrix.push_back(vArray);
	}
	
	//unvisited node
	for (int i = 0; i != m_vUnVisitNodeIdx.size(); ++i) {
		int iSourceIdx = m_vUnVisitNodeIdx[i];
		pcl::PointXYZ oSourceCenter = m_vUnVisitCenters[i];

		for(int j = 0; j != m_vUnVisitNodeIdx.size(); ++j) {
			
			//compute the real cost using the given cost function
			int iTargetIdx = m_vUnVisitNodeIdx[j];
			pcl::PointXYZ oTargetCenter = m_vUnVisitCenters[j];

			//cost function which considers the reward and cost of node
			vEffectMatrix[i][j] = ObjectiveFunction(vReWardMap, iSourceIdx, oSourceCenter,
				                                                iTargetIdx, oTargetCenter);
		}//end if(vUnVisitedStatus[i])
	}
	std::cout << std::endl;

	BBSolver.ObjectiveMatrix(vEffectMatrix);
	std::vector<int> vResTour;
	//output without the frist element, the frist one of output is the goal(next best node)
	float bestCost = BBSolver.SolveOP(vResTour);  //起点定为1，从第二层开始  
	std::cout << "best cost is " << bestCost << std::endl;

	//new plan of nodes
	std::vector<int> vNewPlan;
	std::vector<pcl::PointXYZ> vNewPlanCenters;
	std::vector<bool> vNewPlanWides;
	
	for (int i = 1; i != vResTour.size(); ++i) {//start from 1
	
		int iBBNodeidx = vResTour[i];//note that bb node is beginning from 1 
		//find the real index in unvisited nodes
		int iNodeIdx = m_vUnVisitNodeIdx[iBBNodeidx];
	    pcl::PointXYZ oNodeCenter = m_vUnVisitCenters[iBBNodeidx];

		//save the selected one
		vNewPlan.push_back(iNodeIdx);
		vNewPlanCenters.push_back(oNodeCenter);
		vNewPlanWides.push_back(m_vUnVisitedWide[iBBNodeidx]);

		std::cout << iNodeIdx << " with "<<vReWardMap[iNodeIdx].boundary << " -> " << std::endl;
	}

	std::cout << std::endl;
	 //clear old unvisted node
	m_vUnVisitNodeIdx.clear();
	m_vUnVisitCenters.clear();
	m_vUnVisitedWide.clear();
	m_vUnVisitNodeIdx.resize(vNewPlan.size());
	m_vUnVisitCenters.resize(vNewPlanCenters.size());
	m_vUnVisitedWide.resize(vNewPlanWides.size());
	//save new unvisited node
	//the traversing order is based on the serial number
	//new plan of nodes
	for (int i = 0; i != vNewPlan.size(); ++i) {
		m_vUnVisitNodeIdx[i] = vNewPlan[i];
		m_vUnVisitCenters[i] = vNewPlanCenters[i];
		m_vUnVisitedWide[i] = vNewPlanWides[i];
	}

	return false;

}


void OP::OutputVisitedNodes(std::vector<int> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.resize(m_vVisitedNodeIdx.size());

	//assignment
	for (int i = 0; i != m_vVisitedNodeIdx.size();++i){
		vOutputNodes.push_back(m_vVisitedNodeIdx[i]);
	}

}

int OP::OutputVisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes,
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

void OP::OutputUnVisitedNodes(std::vector<int> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.resize(m_vUnVisitNodeIdx.size());

	//assignment
	for (int i = 0; i != m_vUnVisitNodeIdx.size();++i){
		vOutputNodes.push_back(m_vUnVisitNodeIdx[i]);
	}

}


void OP::OutputUnVisitedNodes(std::vector<pcl::PointXYZ> & vOutputNodes){

	vOutputNodes.clear();
	vOutputNodes.reserve(m_vUnVisitCenters.size());

	//assignment with center value of grid
	for (int i = 0; i != m_vUnVisitCenters.size(); ++i) {		
		vOutputNodes.push_back(m_vUnVisitCenters[i]);
	}

}