
////***********************************************************
////            Next best viewpoint based approaches
////*************************************************************
#include "HpdPointCloudDisplay.h"
#include "LasOperator.h"
#include "readtxt.h"
#include "GridMap.h"
#include "Confidence.h"
#include "GHPR.h"
#include "TSP.h"
#include <iostream>
#include <fstream>
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
	//int iOriViewIdx = 75000;
	//original robot position
	pcl::PointXYZ oRobot;
	//oRobot.x = pAllTravelCloud->points[iOriViewIdx].x;
	//oRobot.y = pAllTravelCloud->points[iOriViewIdx].y;
	//oRobot.z = pAllTravelCloud->points[iOriViewIdx].z + ROBOT_HEIGHT;
	oRobot.x = 0.0;
	oRobot.y = 0.0;
	oRobot.z = ROBOT_HEIGHT;

	//***************generate the grid map for point clouds***********
	std::vector<std::vector<int>> vGridBoundPsIdx;
	std::vector<std::vector<int>> vGridTravelPsIdx;
	std::vector<std::vector<int>> vGridObsPsIdx;

	GridMap oGridMaper(0.5, 500.0, ROBOT_AFFECTDIS,0.6);
	oGridMaper.InitializeMap();


	Confidence oCofSolver(ROBOT_AFFECTDIS);


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
	oGridMaper.m_vReWardMap[iRobotGridIdx].iLabel = 2;
	//*************Compute the real region*************
	//**********The initial point cloud***********
	
	bool bOverFlag = true;
	int iLoopCount = 0;

	pcl::PointXYZ questionPoint;
	questionPoint.x = 0.357257;
	questionPoint.y = 0.247594;
	questionPoint.z = -1.062520;
	
	//find scanning region of each node (site)
	//while(iLoopCount!=10 && bOverFlag){
    while(bOverFlag){
		//judgement
		bOverFlag = false;
	    //robot location
		if(iLoopCount){
		   iRobotGridIdx = OLTSPSolver.m_vUnVisitNodeIdx[0];
		   oRobot.x =  OLTSPSolver.m_vUnVisitCenters[0].x;
		   oRobot.y =  OLTSPSolver.m_vUnVisitCenters[0].y;
		   oRobot.z =  OLTSPSolver.m_vUnVisitCenters[0].z;
		}
		std::cout << "iRobotGridIdx: "<< iRobotGridIdx << std::endl;
		//find scanning region
	    std::vector<int> vNearbyGrids = oGridMaper.SearchGrids(iRobotGridIdx, ROBOT_AFFECTDIS/0.5);
		//get the known grid indexes
	    for (int i = 0; i != vNearbyGrids.size(); ++i)
		     oGridMaper.m_vReWardMap[vNearbyGrids[i]].bKnownFlag = true;
		oGridMaper.RegionGrow(vNearbyGrids);

		oCofSolver.DistanceTerm(oGridMaper.m_vReWardMap, oRobot, vNearbyGrids, *pAllTravelCloud, vGridTravelPsIdx);
		oCofSolver.OcclusionTerm(oGridMaper.m_vReWardMap, oRobot, vNearbyGrids, *pAllTravelCloud, vGridTravelPsIdx);
		oCofSolver.ComputeTotalCoffidence(oGridMaper.m_vReWardMap, vNearbyGrids);
	
		//using the minimum suppression
	    std::vector<int> vNodeGridIdxs = oGridMaper.NonMinimumSuppression();
		
		//OLTSP calculation
	    OLTSPSolver.GetCurrentLocation(pAllTravelCloud, vGridTravelPsIdx, iRobotGridIdx);
	    OLTSPSolver.GetNewNodes(pAllTravelCloud,vGridTravelPsIdx,vNodeGridIdxs);
	    OLTSPSolver.GTR(oGridMaper.m_vReWardMap);

	    iLoopCount = iLoopCount + 1;
	    std::cout << "loops "<< iLoopCount << std::endl;

		if (OLTSPSolver.m_vUnVisitNodeIdx.size())
			bOverFlag = true;

	}//while

	//***************output***************

	//ouput file
	//std::ofstream oRecordedFile;
	//oRecordedFile.open("res.txt", std::ios::out | std::ios::app);

	//display
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

				if(oGridMaper.m_vReWardMap[i].travelable == 1){
				    pKnownCloud->points.push_back(pAllCloud->points[vAllTravelIdx[vGridTravelPsIdx[i][j]]]);
				    vConfidenceValue.push_back(oGridMaper.m_vReWardMap[i].totalValue);
					//oRecordedFile << pAllCloud->points[vAllTravelIdx[vGridTravelPsIdx[i][j]]].x << " "
					//	          << pAllCloud->points[vAllTravelIdx[vGridTravelPsIdx[i][j]]].y << " "
					//	          << pAllCloud->points[vAllTravelIdx[vGridTravelPsIdx[i][j]]].z << " "
					//	          << oGridMaper.m_vReWardMap[i].totalValue << " "
					//	          << std::endl;
				}else{
					pBackgroundCloud->points.push_back(pAllCloud->points[vAllTravelIdx[vGridTravelPsIdx[i][j]]]);
					vBGLabels.push_back(0);
				}

			}
			for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j) {
				pBackgroundCloud->points.push_back(pAllCloud->points[vAllBoundIdx[vGridBoundPsIdx[i][j]]]);
				vBGLabels.push_back(0);
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

	std::vector<int> vLabels(pAllCloud->points.size(),0);
	for(int i=0;i!= oGridMaper.m_vReWardMap.size();++i){
		if(oGridMaper.m_vReWardMap[i].travelable==1){
			for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j)
		        vLabels[vAllTravelIdx[vGridTravelPsIdx[i][j]]] = 1;
	        for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j)
		        vLabels[vAllBoundIdx[vGridBoundPsIdx[i][j]]] = 1;
	        for (int j = 0; j != vGridObsPsIdx[i].size(); ++j)
		        vLabels[vAllObstacleIdx[vGridObsPsIdx[i][j]]] = 1;
		
		}
		
	}


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
	//viewer = hpdisplay.Showclassification(pAllCloud, vLabels,"assign");
	
	//add simulated robot point for display
	for (int i = 0; i != vViewPoints.size(); ++i) {
		stringstream viewpointstream;
		viewpointstream << i << "th_view";
		std::string numlabel;
		viewpointstream >> numlabel;
		stringstream arrowstream;
		arrowstream << i <<"_"<<i+1<<"_arrow";
		std::string arrowNumLabel;
		arrowstream >> arrowNumLabel;
		vViewPoints[i].z = vViewPoints[i].z + ROBOT_HEIGHT;
		viewer->addSphere(vViewPoints[i], 0.2, float(i + 1) / float(vViewPoints.size()), float(i + 1) / float(vViewPoints.size()), float(i + 1) / float(vViewPoints.size()), numlabel.c_str());
		if(i)
		viewer->addArrow(vViewPoints[i], vViewPoints[i-1], 0.0, 0.0, 1.0, false, arrowNumLabel.c_str());
	}

	for (int i = 0; i !=vUnVisitedView.size(); ++i) {
		stringstream unviewpointstream;
		unviewpointstream << i << "th_UnvisitedView";
		std::string unviewpointnumlabel;
		unviewpointstream >> unviewpointnumlabel;

		stringstream unarrowstream;
		unarrowstream << i << "_" << i + 1 << "_unArrow";
		std::string unArrowNumLabel;
		unarrowstream >> unArrowNumLabel;

		vUnVisitedView[i].z = vUnVisitedView[i].z + ROBOT_HEIGHT;
		viewer->addSphere(vUnVisitedView[i], 0.2, 1.0, 0.0, 0.0, unviewpointnumlabel.c_str());
		if (i)
		viewer->addArrow(vUnVisitedView[i], vUnVisitedView[i - 1], 1.0, 0.0, 0.0, false, unArrowNumLabel.c_str());
		else
		viewer->addArrow(vUnVisitedView[i], vViewPoints[vViewPoints.size()-1], 1.0, 0.0, 0.0, false, unArrowNumLabel.c_str());
	}

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


////output
//std::ofstream oRecordedFile;
//oRecordedFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);
//
//for (int i = 0; i != pCloud->points.size(); ++i) {
//
//	//output in a txt file
//	//the storage type of output file is x y z time frames right/left_sensor
//	oRecordedFile << pCloud->points[i].x << " "
//		<< pCloud->points[i].y << " "
//		<< pCloud->points[i].z << " "
//		<< vRes[i] << " "
//		<< oStamp << " "
//		<< std::endl;
//}//end for         
//
//oRecordedFile.close();
//
//}