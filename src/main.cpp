
////***********************************************************
////            Next best viewpoint based approaches
////*************************************************************
#include "LocalPathOptimization.h"
#include "HpdPointCloudDisplay.h"
#include "CurveFitting.h"
#include "LasOperator.h"
#include "Confidence.h"
#include "Evaluation.h"
#include "readtxt.h"
#include "GridMap.h"
#include "Astar.h"
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


	GridMap oGridMaper(0.5, 500.0, ROBOT_AFFECTDIS*0.6, 0.8);
	oGridMaper.InitializeMap();


	Confidence oCofSolver(ROBOT_AFFECTDIS,4.2,5);

	//define output
	std::vector<pcl::PointXYZ> vAstarTrajectory;

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

	std::vector<std::vector<int>> vAStarMap(oGridMaper.m_iGridNum);
	for (int i = 0; i != oGridMaper.m_iGridNum; ++i) {
		for(int j = 0; j != oGridMaper.m_iGridNum; ++j)
		    vAStarMap[i].push_back(1);
	}
	//TSP class
	TSP OLTSPSolver;
	Evaluation oEvaluator;

	int iRobotGridIdx = oGridMaper.AssignPointToMap(oRobot);
	OLTSPSolver.GetNewNode(pAllTravelCloud, vGridTravelPsIdx, iRobotGridIdx);
	oGridMaper.m_vReWardMap[iRobotGridIdx].travelable = 1;
	oGridMaper.m_vReWardMap[iRobotGridIdx].iLabel = 2;
	//*************Compute the real region*************
	//**********The initial point cloud***********
	
	bool bOverFlag = true;
	int iLoopCount = 0;
	
	float fTotalMovingCost = 0.0;

	//find scanning region of each node (site)
	//while(iLoopCount!=10 && bOverFlag){
    while(bOverFlag && fTotalMovingCost <= 200.0){
		//judgement
		bOverFlag = false;
	    //robot location
		if(iLoopCount){
		   iRobotGridIdx = OLTSPSolver.m_vUnVisitNodeIdx[0];
		   oRobot.x =  OLTSPSolver.m_vUnVisitCenters[0].x;
		   oRobot.y =  OLTSPSolver.m_vUnVisitCenters[0].y;
		   oRobot.z =  OLTSPSolver.m_vUnVisitCenters[0].z + ROBOT_HEIGHT;
		}
		
		//find scanning region
	    std::vector<int> vNearbyGrids = oGridMaper.SearchGrids(iRobotGridIdx, ROBOT_AFFECTDIS/0.5);

		//get the scanning point from neighboring region
		pcl::PointCloud<pcl::PointXYZ>::Ptr pScanCloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<ScanIndex> vScanPointIdx;
		
		GetScanPoints(pScanCloud, vScanPointIdx, vNearbyGrids, oGridMaper,
			          vGridBoundPsIdx,vGridTravelPsIdx, vGridObsPsIdx, 
			          pAllObstacleCloud, pAllBoundCloud, pAllTravelCloud);

		//generate a GHPR object
		GHPR oGHPRer(3.8);
		
		//compute the visibility of point clouds
		std::vector<int> vVisableIdx = oGHPRer.ComputeVisibility(*pScanCloud, oRobot);
	
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
		std::cout << "***Compute travelable region success***" << std::endl;

		oCofSolver.DisBoundTerm(oGridMaper.m_vReWardMap, oRobot, vNearbyGrids,
			        *pAllTravelCloud, vGVTravelPsIdx,*pAllBoundCloud, vGVBoundPsIdx);
		std::cout << "***Compute distance term success***" << std::endl;

		std::vector<pcl::PointXYZ> vVisibilityViews;
		std::vector<pcl::PointXYZ> vCurVisitedViews;
		
		OLTSPSolver.OutputVisitedNodes(vCurVisitedViews, pAllTravelCloud, vGridTravelPsIdx);
		

		for (int i = int(vCurVisitedViews.size()) - 1; i >= 0 && i>= int(vCurVisitedViews.size())-1;--i) {
		
			pcl::PointXYZ oOnePoint;
			oOnePoint.x = vCurVisitedViews[i].x;
			oOnePoint.y = vCurVisitedViews[i].y;
			oOnePoint.z = vCurVisitedViews[i].z + ROBOT_HEIGHT;
			vVisibilityViews.push_back(oOnePoint);
		
		}

		//visibility term
		if(vVisibilityViews.size()){
		oCofSolver.OcclusionTerm(oGridMaper.m_vReWardMap, vNearbyGrids, vVisibilityViews,
			                     *pAllTravelCloud, vGVTravelPsIdx, *pAllBoundCloud,
			                     vGVBoundPsIdx,*pAllObstacleCloud, vGVObsPsIdx);
        }
		std::cout << "***Compute visible term success***" << std::endl;

		//quality term
		PathOptimization oPathOptimer;
		for(int i = 0; i != vNearbyGrids.size(); ++i){
			//if it has been scanned
			if (oGridMaper.m_vReWardMap[vNearbyGrids[i]].bKnownFlag){
				//if it is a obstacle grid
				if(oGridMaper.m_vReWardMap[vNearbyGrids[i]].iLabel == 1 ||
				oGridMaper.m_vReWardMap[vNearbyGrids[i]].iLabel == 3){
					//if has not been computed
					if(oGridMaper.m_vReWardMap[vNearbyGrids[i]].Hausdorffflag){
					   //
				       std::vector<int> vNearbyQualityGrids = oGridMaper.SearchGrids(vNearbyGrids[i], 4.0);
					   //compute the quality by using hasudorff
			           oCofSolver.QualityTerm(oGridMaper.m_vReWardMap, vNearbyQualityGrids, *pAllBoundCloud,
					                           vGVBoundPsIdx, *pAllObstacleCloud, vGVObsPsIdx);

					   if (oGridMaper.m_vReWardMap[vNearbyGrids[i]].quality < 1.2) {
					       QualityPair oQualityPair;
					       oQualityPair.idx = vNearbyGrids[i];
					       oQualityPair.quality = oGridMaper.m_vReWardMap[vNearbyGrids[i]].quality;
					       oPathOptimer.m_vControls.push_back(oQualityPair);
                       }
					   for(int i=0;i!=vNearbyQualityGrids.size();++i)
						   oGridMaper.m_vReWardMap[vNearbyQualityGrids[i]].Hausdorffflag = false;

					}//end if oGridMaper.m_vReWardMap[vNearbyGrids[i]].Hausdorffflag
				}//end if
			}//end if
		}//end for int i
		std::cout << "***Compute quality by using hausdorff success***" << std::endl;

		for(int i = 0; i != vNearbyGrids.size(); ++i)
			oGridMaper.m_vReWardMap[vNearbyGrids[i]].Hausdorffflag = true;


		//compute the total confidence
		oCofSolver.ComputeTotalCoffidence(oGridMaper.m_vReWardMap, vNearbyGrids);

		//using the minimum suppression
		std::vector<int> vNodeGridIdxs = oGridMaper.NonMinimumSuppression();
		
		//OLTSP calculation
		OLTSPSolver.GetCurrentLocation(pAllTravelCloud, vGVTravelPsIdx, iRobotGridIdx);
		OLTSPSolver.GetNewNodes(pAllTravelCloud, vGVTravelPsIdx, vNodeGridIdxs);
		OLTSPSolver.GTR(oGridMaper.m_vReWardMap);

		iLoopCount = iLoopCount + 1;
		std::cout << "loops " << iLoopCount << std::endl;

		if (OLTSPSolver.m_vUnVisitNodeIdx.size())
			bOverFlag = true;

		//***********************Local path optimal**********************
		
		if (bOverFlag) {

			//assignment
			for (int i = 0; i != oGridMaper.m_vReWardMap.size(); ++i) {

				int iQuerySX;
				int iQuerySY;
				oGridMaper.IntoXYSeries(iQuerySX, iQuerySY, i);

				if (oGridMaper.m_vReWardMap[i].travelable == 1)
					vAStarMap[iQuerySX][iQuerySY] = 0;
				else
					vAStarMap[iQuerySX][iQuerySY] = 1;

			}

			Astar astar;
			astar.InitAstar(vAStarMap);

			//set the beginning and ending of local path (moving between two sites)			
			int iLclPthPntSX;//
			int iLclPthPntSY;
			oGridMaper.IntoXYSeries(iLclPthPntSX, iLclPthPntSY, OLTSPSolver.m_vVisitedNodeIdx[OLTSPSolver.m_vVisitedNodeIdx.size() - 1]);
			GridNode oLocalPathStart(iLclPthPntSX, iLclPthPntSY);
			oGridMaper.IntoXYSeries(iLclPthPntSX, iLclPthPntSY, OLTSPSolver.m_vUnVisitNodeIdx[0]);
			GridNode oLocalPathEnd(iLclPthPntSX, iLclPthPntSY);

			//A-star is to get the paths
			
			list<GridNode *> vLocalPath = astar.GetPath(oLocalPathStart, oLocalPathEnd, false);
	
			pcl::PointCloud<pcl::PointXYZ>::Ptr pAnchors(new pcl::PointCloud<pcl::PointXYZ>);
			//save the trajectory
			for (auto &pPathPoints : vLocalPath) {

				int iQueryAstarIdx = oGridMaper.ComputeGridIdx(pPathPoints->x, pPathPoints->y);
				pcl::PointXYZ oAStarPoint = TSP::ComputeCentersPosition(pAllTravelCloud, vGVTravelPsIdx, iQueryAstarIdx);
				pAnchors->points.push_back(oAStarPoint);
				//vAstarTrajectory.push_back(oAStarPoint);
			}

			oPathOptimer.GetControlCenter(pAllObstacleCloud, vGVObsPsIdx);
		
			oPathOptimer.NewLocalPath(pAnchors, oGridMaper,1.5);
			std::cout << "***Compute local path success***" << std::endl;

			//spline optimal
			Spline oBezierLine;
			std::vector<std::vector<pcl::PointXYZ>> vOneBezierSpline;
			std::vector<pcl::PointXYZ> vOneLocalPath;
			

			oBezierLine.NonSingularityBezierSplineGeneration(vOneBezierSpline, pAnchors, 0.01);
			oBezierLine.TransforContinuousSpline(vOneLocalPath, vOneBezierSpline);
			std::cout << "***Compute spline success***" << std::endl;

			//Evaluate the cost 

			oEvaluator.AddDistanceValue(oEvaluator.ComputeMovingDis(vOneLocalPath));
			oEvaluator.Output(fTotalMovingCost);
			std::cout << "Total moving distance: "<< fTotalMovingCost << std::endl;

			for(int i = 0; i != vOneLocalPath.size(); ++i)
				vAstarTrajectory.push_back(vOneLocalPath[i]);

        }//if bOverFlag

	}//while

	//***************output***************

	//ouput file
	std::ofstream oCloudOutFile;
	oCloudOutFile.open("ScannedPointClouds.txt", std::ios::out | std::ios::app);

	//display
	pcl::PointCloud<pcl::PointXYZ>::Ptr pKnownCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> vConfidenceValue;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pBackgroundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> vBGLabels;
	

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
					oCloudOutFile   << pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].x << " "
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
					oCloudOutFile << pAllCloud->points[vAllTravelIdx[vGVTravelPsIdx[i][j]]].x << " "
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
				oCloudOutFile << pAllCloud->points[vAllBoundIdx[vGVBoundPsIdx[i][j]]].x << " "
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
				oCloudOutFile << pAllCloud->points[vAllObstacleIdx[vGVObsPsIdx[i][j]]].x << " "
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

	std::ofstream oTrajectoryFile;
	oTrajectoryFile.open("Trajectory.txt", std::ios::out | std::ios::app);
	//add simulated robot point for display
	for (int i = 0; i != vAstarTrajectory.size(); ++i) {
	
		vAstarTrajectory[i].z = vAstarTrajectory[i].z + ROBOT_HEIGHT;
		oTrajectoryFile << vAstarTrajectory[i].x << " "
			            << vAstarTrajectory[i].y << " "
			            << vAstarTrajectory[i].z << " " 
		              	<< i << " " << std::endl;
		//if (i)
		//	viewer->addArrow(vAstarTrajectory[i], vAstarTrajectory[i - 1], 0.0, 0.0, 1.0, false, arrowNumLabel.c_str());
	}

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
	
	//add simulated robot point for display
	for (int i = 0; i < vAstarTrajectory.size(); i = i + 20) {
		stringstream viewpointstream;
		viewpointstream << i << "th_view";
		std::string numlabel;
		viewpointstream >> numlabel;
		stringstream arrowstream;
		arrowstream << i << "_" << i + 1 << "_arrow";
		std::string arrowNumLabel;
		arrowstream >> arrowNumLabel;
		vAstarTrajectory[i].z = vAstarTrajectory[i].z + ROBOT_HEIGHT;
		viewer->addSphere(vAstarTrajectory[i], 0.2, float(i + 1) / float(vAstarTrajectory.size()), float(i + 1) / float(vAstarTrajectory.size()), float(i + 1) / float(vAstarTrajectory.size()), numlabel.c_str());
		
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
		viewer->addSphere(vUnVisitedView[i], 0.1, 1.0, 0.0, 0.0, unviewpointnumlabel.c_str());
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
//std::ofstream oCloudOutFile;
//oCloudOutFile.open(m_sOutPCFileName.str(), std::ios::out | std::ios::app);
//
//for (int i = 0; i != pCloud->points.size(); ++i) {
//
//	//output in a txt file
//	//the storage type of output file is x y z time frames right/left_sensor
//	oCloudOutFile << pCloud->points[i].x << " "
//		<< pCloud->points[i].y << " "
//		<< pCloud->points[i].z << " "
//		<< vRes[i] << " "
//		<< oStamp << " "
//		<< std::endl;
//}//end for         
//
//oCloudOutFile.close();
//
//}