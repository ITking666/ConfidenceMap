
////***********************************************************
////            Next best viewpoint based approaches
////*************************************************************
#include "HpdPointCloudDisplay.h"
#include "LasOperator.h"
#include "readtxt.h"
#include "GridMap.h"
#include "Confidence.h"
#include "HPR.h"
#include "TSP.h"
#include <iostream>
#include <cmath>
#include <ctime>
#define ROBOT_HEIGHT 1.13933



int main() {


	//***************************Read labelled point clouds*************************************!!!
	//read data
	std::vector<std::vector<double> > vRawData;
	//read data for data file
	ReadMatrix("test2.txt", vRawData);
	std::cout << "Reading data completed!" << std::endl;

	//all point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr pAllCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vAllPointLabels;

	//each label point
	pcl::PointCloud<pcl::PointXYZ>::Ptr pAllObstacleCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vAllObstacleIdx;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pAllBoundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vAllBoundIdx;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pAllTravelCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vAllTravelIdx;
	
	//read data
	for (size_t i = 0; i != vRawData.size(); ++i) {

		//read point
		pcl::PointXYZ oPoint;
		oPoint.x = vRawData[i][0];
		oPoint.y = vRawData[i][1];
		oPoint.z = vRawData[i][2];
		//whole point clouds
		pAllCloud->points.push_back(oPoint);

		//each point clouds
		vAllPointLabels.push_back(vRawData[i][3]);
		
		//take index for inital robot
		if (vRawData[i][3] == 1) {//travelable part
			pAllTravelCloud->points.push_back(oPoint);
			vAllTravelIdx.push_back(i);
		
		}else if (vRawData[i][3] == 2) {//boundary part
			pAllBoundCloud->points.push_back(oPoint);
			vAllBoundIdx.push_back(i);
		}
		else {//obstacle part
			pAllObstacleCloud->points.push_back(oPoint);
			vAllObstacleIdx.push_back(i);
		}

	}
	
	//view points

	//**************simulated robot point*******************
	//original view point index based on ground points
	int iOriViewIdx = 75000;
	//original robot position
	pcl::PointXYZ oRobot;
	oRobot.x = pAllTravelCloud->points[iOriViewIdx].x;
	oRobot.y = pAllTravelCloud->points[iOriViewIdx].y;
	oRobot.z = pAllTravelCloud->points[iOriViewIdx].z + ROBOT_HEIGHT;
	
	//***************generate the grid map for point clouds***********
	std::vector<std::vector<int>> vGridBoundPsIdx;
	std::vector<std::vector<int>> vGridTravelPsIdx;
	std::vector<std::vector<int>> vGridObsPsIdx;

	GridMap oGridMaper(0.5, 500.0,15.0,0.2);
	oGridMaper.InitializeMap();

	Confidence oCofSolver(oGridMaper.m_vReWardMap.size());

	//map
	oGridMaper.GenerateMap(vGridTravelPsIdx);
	oGridMaper.GenerateMap(vGridBoundPsIdx);
	oGridMaper.GenerateMap(vGridObsPsIdx);
	
	//
	oGridMaper.AssignPointsToMap(*pAllObstacleCloud, vGridObsPsIdx, 1);
	oGridMaper.AssignPointsToMap(*pAllTravelCloud, vGridTravelPsIdx,2);
	oGridMaper.AssignPointsToMap(*pAllBoundCloud, vGridBoundPsIdx,3);

	//TSP class
	TSP OLTSPSolver;
 
	int iRobotGridIdx = oGridMaper.AssignPointToMap(oRobot);
	OLTSPSolver.GetNewNode(pAllTravelCloud, vGridTravelPsIdx, iRobotGridIdx);
	oGridMaper.m_vReWardMap[iRobotGridIdx].travelable = 1;
	//*************Compute the real region*************
	//**********The initial point cloud***********
	
	bool bOverFlag = true;
	int iLoopCount = 0;

	//find scanning region of each node (site)
	while(iLoopCount!=1&&bOverFlag){
		//judgement
		bOverFlag = false;
	    //robot location
		if(iLoopCount)
		   iRobotGridIdx = OLTSPSolver.m_vUnVisitNodeIdx[0];
		 
		std::cout << "iRobotGridIdx: "<< iRobotGridIdx << std::endl;
		//find scanning region
	    std::vector<int> vNearbyGrids = oGridMaper.SearchGrids(iRobotGridIdx, ROBOT_AFFECTDIS/0.5);
		//get the known grid indexes
	    for (int i = 0; i != vNearbyGrids.size(); ++i)
		     oGridMaper.m_vReWardMap[vNearbyGrids[i]].bKnownFlag = true;
		oGridMaper.RegionGrow(vNearbyGrids);

		//compute the features
	    for (int i = 0; i != vNearbyGrids.size(); ++i) {
		//**************Confidence Map*******************
		     std::vector<int> vNearbyJudge = oGridMaper.SearchGrids(vNearbyGrids[i], 0.5);
		     oCofSolver.FrontierTerm(oGridMaper.m_vReWardMap, vNearbyGrids[i], vNearbyJudge);
		     oCofSolver.ComputeTotalCoffidence(oGridMaper.m_vReWardMap, vNearbyGrids[i]);
	    }
	
		//using the minimum suppression
	    std::vector<int> vNodeGridIdxs = oGridMaper.NonMinimumSuppression();

		//OLTSP calculation
	    OLTSPSolver.GetCurrentLocation(pAllTravelCloud, vGridTravelPsIdx, iRobotGridIdx);
	    OLTSPSolver.GetNewNodes(pAllTravelCloud,vGridTravelPsIdx,vNodeGridIdxs);
	    OLTSPSolver.GTR(oGridMaper.m_vReWardMap);

		std::cout << "the number of new nodes: " << vNodeGridIdxs.size() << std::endl;

	    iLoopCount = iLoopCount + 1;
	    std::cout << "loops "<< iLoopCount << std::endl;

		std::cout << "size: " << OLTSPSolver.m_vUnVisitNodeIdx.size() << std::endl;
		if (OLTSPSolver.m_vUnVisitNodeIdx.size())
			bOverFlag = true;

	}//while

	 //output
	pcl::PointCloud<pcl::PointXYZ>::Ptr pKnownCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> vConfidenceValue;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pBackgroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vBGLabels;

	for (int i = 0; i != oGridMaper.m_vReWardMap.size(); ++i) {

		if (oGridMaper.m_vReWardMap[i].bKnownFlag) {

			for (int j = 0; j != vGridObsPsIdx[i].size(); ++j) {
				pBackgroundCloud->points.push_back(pAllCloud->points[vAllObstacleIdx[vGridObsPsIdx[i][j]]]);
				vBGLabels.push_back(0);
			}
			for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j) {

				pKnownCloud->points.push_back(pAllCloud->points[vAllTravelIdx[vGridTravelPsIdx[i][j]]]);
				vConfidenceValue.push_back(oGridMaper.m_vReWardMap[i].totalValue);
			}
			for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j) {
				pBackgroundCloud->points.push_back(pAllCloud->points[vAllBoundIdx[vGridBoundPsIdx[i][j]]]);
				vBGLabels.push_back(1);
			}
		}//end if
	}
	//std::vector<int> vLabels(pAllCloud->points.size(),0);
	//for (int i = 0; i != oGridMaper.m_vReWardMap.size(); ++i) {

	//	if (oGridMaper.m_vReWardMap[i].travelable == 1) {
	//		for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j)
	//			vLabels[vAllTravelIdx[vGridTravelPsIdx[i][j]]] = 1;
	//		for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j)
	//			vLabels[vAllBoundIdx[vGridBoundPsIdx[i][j]]] = 1;
	//		for (int j = 0; j != vGridObsPsIdx[i].size(); ++j)
	//			vLabels[vAllObstacleIdx[vGridObsPsIdx[i][j]]] = 1;
	//	}
	//}




	std::vector<pcl::PointXYZ> vViewPoints;
	std::vector<pcl::PointXYZ> vUnVisitedView;
	//OLTSPSolver.OutputUnVisitedNodes(vViewPoints);
	OLTSPSolver.OutputVisitedNodes(vViewPoints, pAllTravelCloud, vGridTravelPsIdx);
	OLTSPSolver.OutputUnVisitedNodes(vUnVisitedView);
	//*****************show result*******************	
	//save the result
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	HpdDisplay hpdisplay;

	viewer = hpdisplay.ShowMixedResult(pKnownCloud, vConfidenceValue,
		                               pBackgroundCloud, vBGLabels,
		                               "redgreen", "assign");
	//viewer = hpdisplay.Showclassification(pAllCloud, vLabels);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}

	return 0;

}


//*******************************************************************
//                      show region
//*******************************************************************
//for (int i = 0; i != vNearbyGrids.size(); ++i) {
//
//	for (int j = 0; j != vGridTravelPsIdx[vNearbyGrids[i]].size(); ++j)
//		labels[vAllTravelIdx[vGridTravelPsIdx[vNearbyGrids[i]][j]]] = 1;
//	for (int j = 0; j != vGridBoundPsIdx[vNearbyGrids[i]].size(); ++j)
//		labels[vAllBoundIdx[vGridBoundPsIdx[vNearbyGrids[i]][j]]] = 1;
//	for (int j = 0; j != vGridObsPsIdx[vNearbyGrids[i]].size(); ++j)
//		labels[vAllObstacleIdx[vGridObsPsIdx[vNearbyGrids[i]][j]]] = 1;
//
//}
//for (int j = 0; j != vGridTravelPsIdx[iRobotGridIdx].size(); ++j)
//labels[vAllTravelIdx[vGridTravelPsIdx[iRobotGridIdx][j]]] = 2;
//for (int j = 0; j != vGridBoundPsIdx[iRobotGridIdx].size(); ++j)
//labels[vAllBoundIdx[vGridBoundPsIdx[iRobotGridIdx][j]]] = 2;
//for (int j = 0; j != vGridObsPsIdx[iRobotGridIdx].size(); ++j)
//labels[vAllObstacleIdx[vGridObsPsIdx[iRobotGridIdx][j]]] = 2;

//*******************************************************************
//                      show label
//*******************************************************************
//for (int i = 0; i != oGridMaper.m_vReWardMap.size(); ++i) {
//
//	for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j) {
//		labels[vAllTravelIdx[vGridTravelPsIdx[i][j]]] = oGridMaper.m_vReWardMap[i].iLabel;
//
//	}
//	for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j) {
//
//		labels[vAllBoundIdx[vGridBoundPsIdx[i][j]]] = oGridMaper.m_vReWardMap[i].iLabel;
//
//	}
//	for (int j = 0; j != vGridObsPsIdx[i].size(); ++j) {
//		labels[vAllObstacleIdx[vGridObsPsIdx[i][j]]] = 0;
//
//	}
//}
