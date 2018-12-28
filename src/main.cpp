
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

struct ScanIndex{

	//label = 0 is obstacle cloud
	//label = 1 is ground cloud
	//label = 2 is bound cloud
	short int label;
	int idx;

};

struct PVGridStatus{

	bool visible;
	bool added;

	PVGridStatus(){
		
		visible = false;
		added = false;
	
	};
};

void GetScanPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr & pScanCloud,
                   std::vector<ScanIndex> & vScanPointIdx,
	               const std::vector<int> & vNearbyGrids,
	               const GridMap & oGridMaper,
	               const std::vector<std::vector<int>> & vGridBoundPsIdx,
	               const std::vector<std::vector<int>> & vGridTravelPsIdx,
	               const std::vector<std::vector<int>> & vGridObsPsIdx,
	               const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllObstacleCloud,
	               const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllBoundCloud,
	               const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllTravelCloud){

	for (int i = 0; i != vNearbyGrids.size(); ++i) {

		int iOneGridIdx = vNearbyGrids[i];
		//if it is a ground grid
		if (oGridMaper.m_vReWardMap[vNearbyGrids[i]].iLabel == 2) {

			//record the ground point only
			for (int j = 0; j != vGridTravelPsIdx[iOneGridIdx].size(); ++j) {
				pScanCloud->points.push_back(pAllTravelCloud->points[vGridTravelPsIdx[iOneGridIdx][j]]);
				ScanIndex oOneIndex;
				oOneIndex.label = 1;
				oOneIndex.idx = vGridTravelPsIdx[iOneGridIdx][j];
				vScanPointIdx.push_back(oOneIndex);
			}//end for j

		}
		else {//if it is impossible being a unreachable grid

			  //record the obstacle point
			for (int j = 0; j != vGridObsPsIdx[vNearbyGrids[i]].size(); ++j) {
				pScanCloud->points.push_back(pAllObstacleCloud->points[vGridObsPsIdx[iOneGridIdx][j]]);
				ScanIndex oOneIndex;
				oOneIndex.label = 0;
				oOneIndex.idx = vGridObsPsIdx[iOneGridIdx][j];
				vScanPointIdx.push_back(oOneIndex);

			}
			//record the boundary point
			for (int j = 0; j != vGridBoundPsIdx[vNearbyGrids[i]].size(); ++j) {
				pScanCloud->points.push_back(pAllBoundCloud->points[vGridBoundPsIdx[iOneGridIdx][j]]);
				ScanIndex oOneIndex;
				oOneIndex.label = 2;
				oOneIndex.idx = vGridBoundPsIdx[iOneGridIdx][j];
				vScanPointIdx.push_back(oOneIndex);
			}
			//record the ground point
			for (int j = 0; j != vGridTravelPsIdx[vNearbyGrids[i]].size(); ++j) {
				pScanCloud->points.push_back(pAllTravelCloud->points[vGridTravelPsIdx[iOneGridIdx][j]]);
				ScanIndex oOneIndex;
				oOneIndex.label = 1;
				oOneIndex.idx = vGridTravelPsIdx[iOneGridIdx][j];
				vScanPointIdx.push_back(oOneIndex);
			}
		}//end else

	}//end for

};

void RecordScanLabel(std::vector<PVGridStatus> & vObstacleScan,
                     std::vector<PVGridStatus> & vTravelScan,
                     std::vector<PVGridStatus> & vBoundScan,
	                 const std::vector<ScanIndex> & vScanPointIdx, 
	                 const std::vector<int> & vVisableIdx){
	
	for (int i = 0; i != vVisableIdx.size(); ++i) {

	//if it is a ground
	    if (vScanPointIdx[vVisableIdx[i]].label == 0){
			vObstacleScan[vScanPointIdx[vVisableIdx[i]].idx].visible = true;
			
		}else if (vScanPointIdx[vVisableIdx[i]].label == 1){
			vTravelScan[vScanPointIdx[vVisableIdx[i]].idx].visible = true;
			
		}else{
			vBoundScan[vScanPointIdx[vVisableIdx[i]].idx].visible = true;
			
        }

    }//end for i = 0; i != vVisableIdx.size(); ++i

}

void GetGridVisiblePIdx(std::vector<std::vector<int>> & vGVTravelPsIdx,
                              std::vector<PVGridStatus> & vTravelScan,
	           const std::vector<std::vector<int>> & vGridTravelPsIdx,
                          std::vector<std::vector<int>> & vGVObsPsIdx,
	                        std::vector<PVGridStatus> & vObstacleScan,                   
	              const std::vector<std::vector<int>> & vGridObsPsIdx,
	                    std::vector<std::vector<int>> & vGVBoundPsIdx,
                               std::vector<PVGridStatus> & vBoundScan,
                const std::vector<std::vector<int>> & vGridBoundPsIdx){

	//save visible travel idx
	for (int i = 0; i != vGridTravelPsIdx.size(); ++i) {
		for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j) {
			if (vTravelScan[vGridTravelPsIdx[i][j]].visible &&
				!vTravelScan[vGridTravelPsIdx[i][j]].added) {
				
				vGVTravelPsIdx[i].push_back(vGridTravelPsIdx[i][j]);
				vTravelScan[vGridTravelPsIdx[i][j]].added = true;

			}
		}
	}

	//save the boundary points
	for(int i = 0;i!= vGridBoundPsIdx.size();++i){
		for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j) {
			if (vBoundScan[vGridBoundPsIdx[i][j]].visible &&
				!vBoundScan[vGridBoundPsIdx[i][j]].added) {

				vGVBoundPsIdx[i].push_back(vGridBoundPsIdx[i][j]);
				vBoundScan[vGridBoundPsIdx[i][j]].added = true;

			}
		}
    }

	//save obstacle points
	for (int i = 0; i != vGridObsPsIdx.size(); ++i) {
		for (int j = 0; j != vGridObsPsIdx[i].size(); ++j) {
			if (vObstacleScan[vGridObsPsIdx[i][j]].visible &&
				!vObstacleScan[vGridObsPsIdx[i][j]].added) {

				vGVObsPsIdx[i].push_back(vGridObsPsIdx[i][j]);
				vObstacleScan[vGridObsPsIdx[i][j]].added = true;

			}
		}
	}

};




int main() {


	//***************************Read labelled point clouds*************************************!!!
	//read data
	std::vector<std::vector<double> > vRawData;
	//read data for data file
	ReadMatrix("test.txt", vRawData);
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


	GridMap oGridMaper(0.5, 500.0, ROBOT_AFFECTDIS*0.5, 0.8);
	oGridMaper.InitializeMap();


	Confidence oCofSolver(ROBOT_AFFECTDIS,4.2,5);

	//the scanning label in simulation
	std::vector<PVGridStatus> vObstacleScan(pAllObstacleCloud->points.size());
	std::vector<PVGridStatus> vTravelScan(pAllTravelCloud->points.size());
	std::vector<PVGridStatus> vBoundScan(pAllBoundCloud->points.size());

	//map
	oGridMaper.GenerateMap(vGridTravelPsIdx);
	oGridMaper.GenerateMap(vGridBoundPsIdx);
	oGridMaper.GenerateMap(vGridObsPsIdx);
	std::vector<std::vector<int>> vGVBoundPsIdx(vGridBoundPsIdx.size());
	std::vector<std::vector<int>> vGVTravelPsIdx(vGridTravelPsIdx.size());
	std::vector<std::vector<int>> vGVObsPsIdx(vGridObsPsIdx.size());
	

	//assign point to map
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

	//find scanning region of each node (site)
	//while(iLoopCount!=10 && bOverFlag){
    while(iLoopCount != 1 && bOverFlag){
		//judgement
		bOverFlag = false;
	    //robot location
		if(iLoopCount){
		   iRobotGridIdx = OLTSPSolver.m_vUnVisitNodeIdx[0];
		   oRobot.x =  OLTSPSolver.m_vUnVisitCenters[0].x;
		   oRobot.y =  OLTSPSolver.m_vUnVisitCenters[0].y;
		   oRobot.z =  OLTSPSolver.m_vUnVisitCenters[0].z + ROBOT_HEIGHT;
		}
		std::cout << "iRobotGridIdx: "<< iRobotGridIdx << std::endl;
		//find scanning region
	    std::vector<int> vNearbyGrids = oGridMaper.SearchGrids(iRobotGridIdx, ROBOT_AFFECTDIS/0.5);

		//get the scanning point from neighboring region
		pcl::PointCloud<pcl::PointXYZ>::Ptr pScanCloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<ScanIndex> vScanPointIdx;
		std::cout << "1" << std::endl;
		GetScanPoints(pScanCloud, vScanPointIdx, vNearbyGrids, oGridMaper,
			          vGridBoundPsIdx,vGridTravelPsIdx, vGridObsPsIdx, 
			          pAllObstacleCloud, pAllBoundCloud, pAllTravelCloud);

		//generate a GHPR object
		GHPR oGHPRer(3.8);
		
		//compute the visibility of point clouds
		std::vector<int> vVisableIdx = oGHPRer.ComputeVisibility(*pScanCloud, oRobot);
		std::cout << "2" << std::endl;
		//record the scanning
		RecordScanLabel(vObstacleScan, vTravelScan, vBoundScan,
			                         vScanPointIdx, vVisableIdx);

		//get the grid visible point indices
		GetGridVisiblePIdx(vGVTravelPsIdx, vTravelScan, vGridTravelPsIdx, 
			                   vGVObsPsIdx, vObstacleScan, vGridObsPsIdx,
			                  vGVBoundPsIdx, vBoundScan, vGridBoundPsIdx);
		
		//get the known grid indexes
		for (int i = 0; i != vNearbyGrids.size(); ++i){
			int iQueryIdx = vNearbyGrids[i];
			if(vGVTravelPsIdx[iQueryIdx].size() ||
				  vGVObsPsIdx[iQueryIdx].size() ||
				vGVBoundPsIdx[iQueryIdx].size())
			oGridMaper.m_vReWardMap[vNearbyGrids[i]].bKnownFlag = true;
		}

		oGridMaper.RegionGrow(vNearbyGrids);
		
		oCofSolver.DistanceTerm(oGridMaper.m_vReWardMap, oRobot, vNearbyGrids, *pAllTravelCloud, vGVTravelPsIdx);

		std::vector<pcl::PointXYZ> vVisibilityViews;
		std::vector<pcl::PointXYZ> vCurVisitedViews;
		std::cout << "2.5" << std::endl;
		OLTSPSolver.OutputVisitedNodes(vCurVisitedViews, pAllTravelCloud, vGridTravelPsIdx);
		std::cout <<"visited viewpoint num: "<< vCurVisitedViews.size() << std::endl;
		std::cout <<" vCurVisitedViews.size() - 1: " << vCurVisitedViews.size() - 1 << std::endl;
		std::cout <<" vCurVisitedViews.size() - 2: " << int(vCurVisitedViews.size()) - 2 << std::endl;

		for (int i = int(vCurVisitedViews.size()) - 1; i >= 0 && i>= int(vCurVisitedViews.size())-1;--i) {
		
			pcl::PointXYZ oOnePoint;
			oOnePoint.x = vCurVisitedViews[i].x;
			oOnePoint.y = vCurVisitedViews[i].y;
			oOnePoint.z = vCurVisitedViews[i].z + ROBOT_HEIGHT;
			vVisibilityViews.push_back(oOnePoint);
			std::cout << "got it" << std::endl;
		}

		//visibility term
		if(vVisibilityViews.size()){
		oCofSolver.OcclusionTerm(oGridMaper.m_vReWardMap, vNearbyGrids, vVisibilityViews,
			                     *pAllTravelCloud, vGVTravelPsIdx, *pAllBoundCloud,
			                     vGVBoundPsIdx,*pAllObstacleCloud, vGVObsPsIdx);
        }

		//quality term
		oCofSolver.QualityTerm(oGridMaper.m_vReWardMap, vNearbyGrids,
		                    	*pAllTravelCloud, vGVTravelPsIdx, *pAllBoundCloud,
			                     vGVBoundPsIdx, *pAllObstacleCloud, vGVObsPsIdx);

		//compute the total confidence
		oCofSolver.ComputeTotalCoffidence(oGridMaper.m_vReWardMap, vNearbyGrids);

		//using the minimum suppression
		std::vector<int> vNodeGridIdxs = oGridMaper.NonMinimumSuppression();

		//OLTSP calculation
		OLTSPSolver.GetCurrentLocation(pAllTravelCloud, vGridTravelPsIdx, iRobotGridIdx);
		OLTSPSolver.GetNewNodes(pAllTravelCloud, vGridTravelPsIdx, vNodeGridIdxs);
		OLTSPSolver.GTR(oGridMaper.m_vReWardMap);

		iLoopCount = iLoopCount + 1;
		std::cout << "loops " << iLoopCount << std::endl;

		if (OLTSPSolver.m_vUnVisitNodeIdx.size())
			bOverFlag = true;

	}//while

	//***************output***************

	//ouput file
	std::ofstream oRecordedFile;
	oRecordedFile.open("res.txt", std::ios::out | std::ios::app);

	//display
	pcl::PointCloud<pcl::PointXYZ>::Ptr pKnownCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> vConfidenceValue;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pBackgroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vBGLabels;
	std::cout << "3" << std::endl;

	//for display the result
	for (int i = 0; i != oGridMaper.m_vReWardMap.size(); ++i) {

		//save the travel points
		if (oGridMaper.m_vReWardMap[i].bKnownFlag) {
			//if here is the travelable region 
			if (oGridMaper.m_vReWardMap[i].travelable == 1) {

				for (int j = 0; j != vGVTravelPsIdx[i].size(); ++j) {

					pKnownCloud->points.push_back(pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]]);
					vConfidenceValue.push_back(oGridMaper.m_vReWardMap[i].totalValue);
					//record the data in txt file for test
					oRecordedFile   << pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].x << " "
					            	<< pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].y << " "
					            	<< pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].z << " "
						            << oGridMaper.m_vReWardMap[i].disTermVal << " "
						            << oGridMaper.m_vReWardMap[i].visibility << " "
						            << oGridMaper.m_vReWardMap[i].quality << " "
						            << oGridMaper.m_vReWardMap[i].totalValue << " "
						            << std::endl;

				}//end for j
			
			}else {

				for (int j = 0; j != vGVTravelPsIdx[i].size(); ++j) {

					pBackgroundCloud->points.push_back(pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]]);
					vBGLabels.push_back(0);
					oRecordedFile << pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].x << " "
						<< pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].y << " "
						<< pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].z << " "
						<< oGridMaper.m_vReWardMap[i].disTermVal << " "
						<< oGridMaper.m_vReWardMap[i].visibility << " "
						<< oGridMaper.m_vReWardMap[i].quality << " "
						<< oGridMaper.m_vReWardMap[i].totalValue << " "
						<< std::endl;
				}//end for j
			}//end else

			 //save the boundary points
			for (int j = 0; j != vGVBoundPsIdx[i].size(); ++j) {

				pBackgroundCloud->points.push_back(pAllCloud->points[vAllBoundIdx[vGVBoundPsIdx[i][j]]]);
				vBGLabels.push_back(0);
				oRecordedFile << pAllCloud->points[vAllBoundIdx[vGVBoundPsIdx[i][j]]].x << " "
					<< pAllCloud->points[vAllBoundIdx[vGVBoundPsIdx[i][j]]].y << " "
					<< pAllCloud->points[vAllBoundIdx[vGVBoundPsIdx[i][j]]].z << " "
					<< oGridMaper.m_vReWardMap[i].disTermVal << " "
					<< oGridMaper.m_vReWardMap[i].visibility << " "
					<< oGridMaper.m_vReWardMap[i].quality << " "
					<< oGridMaper.m_vReWardMap[i].totalValue << " "
					<< std::endl;
			}

			//save obstacle points
			for (int j = 0; j != vGVObsPsIdx[i].size(); ++j) {

				pBackgroundCloud->points.push_back(pAllCloud->points[vAllObstacleIdx[vGVObsPsIdx[i][j]]]);
				vBGLabels.push_back(0);
				oRecordedFile << pAllCloud->points[vAllObstacleIdx[vGVObsPsIdx[i][j]]].x << " "
					<< pAllCloud->points[vAllObstacleIdx[vGVObsPsIdx[i][j]]].y << " "
					<< pAllCloud->points[vAllObstacleIdx[vGVObsPsIdx[i][j]]].z << " "
					<< oGridMaper.m_vReWardMap[i].disTermVal << " "
					<< oGridMaper.m_vReWardMap[i].visibility << " "
					<< oGridMaper.m_vReWardMap[i].quality << " "
					<< oGridMaper.m_vReWardMap[i].totalValue << " "
					<< std::endl;
			}

		}//end if (oGridMaper.m_vReWardMap[i].bKnownFlag)
	}

	std::cout << "4" << std::endl;


	//std::vector<int> vLabels(pAllCloud->points.size(),0);
	//for(int i=0;i!= oGridMaper.m_vReWardMap.size();++i){

	//for (int j = 0; j != vGridTravelPsIdx[i].size(); ++j)
	//	if(vTravelScan[vGridTravelPsIdx[i][j]])
	//       vLabels[vAllTravelIdx[vGridTravelPsIdx[i][j]]] = 1;

	//for (int j = 0; j != vGridBoundPsIdx[i].size(); ++j)
	//	if(vBoundScan[vGridBoundPsIdx[i][j]])
	//       vLabels[vAllBoundIdx[vGridBoundPsIdx[i][j]]] = 1;

	//for (int j = 0; j != vGridObsPsIdx[i].size(); ++j)
	//	if(vObstacleScan[vGridObsPsIdx[i][j]])
	//       vLabels[vAllObstacleIdx[vGridObsPsIdx[i][j]]] = 1;

	//}

	std::cout << "5" << std::endl;

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
		arrowstream << i << "_" << i + 1 << "_arrow";
		std::string arrowNumLabel;
		arrowstream >> arrowNumLabel;
		vViewPoints[i].z = vViewPoints[i].z + ROBOT_HEIGHT;
		viewer->addSphere(vViewPoints[i], 0.2, float(i + 1) / float(vViewPoints.size()), float(i + 1) / float(vViewPoints.size()), float(i + 1) / float(vViewPoints.size()), numlabel.c_str());
		if (i)
			viewer->addArrow(vViewPoints[i], vViewPoints[i - 1], 0.0, 0.0, 1.0, false, arrowNumLabel.c_str());
	}

	for (int i = 0; i != vUnVisitedView.size(); ++i) {
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
			viewer->addArrow(vUnVisitedView[i], vViewPoints[vViewPoints.size() - 1], 1.0, 0.0, 0.0, false, unArrowNumLabel.c_str());
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