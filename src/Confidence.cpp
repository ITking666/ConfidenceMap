#include "Confidence.h"



Confidence::Confidence(const int & iGridNum)
{

	MapKnownStatus.resize(iGridNum, false);

}



Confidence::~Confidence(){


}


//Gaussian Kernel Function
inline float Confidence::GaussianKernel(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo, float & sigma){

	// k(|| x - xc || ) = exp{ -|| x - xc || ^ 2 / (2 * sigma^2) }
	//or k(|| x - xc || ) = exp{ -|| x - xc || ^ 2 / (sigma^2) }

	//distance between two input vector
	float fNormSquare = Compute2Norm(oQueryPo, oTargerPo);
	//
	      fNormSquare = pow(fNormSquare, 2.0f);
	//ouput equation result
	return exp(-1.0f * fNormSquare)/pow(sigma,2.0f);

}

//the 2 norm of a vector
inline float Confidence::Compute2Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo){

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPo.x - oTargerPo.x, 2.0f)
		      + pow(oQueryPo.y - oTargerPo.y, 2.0f)
		      + pow(oQueryPo.z - oTargerPo.z, 2.0f));

}

//the 1 norm of a vector
inline float Confidence::Compute1Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo){

	return sqrt(pow(oQueryPo.x - oTargerPo.x, 2.0f)
		      + pow(oQueryPo.y - oTargerPo.y, 2.0f)
	          + pow(oQueryPo.z - oTargerPo.z, 2.0f));

}


pcl::PointXYZ Confidence::ComputeCenter(const PCLCloudXYZ & vCloud, const std::vector<int> & vPointIdx){

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
//reload
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


void Confidence::GetKnownGridIdx(const std::vector<int> & vKnownGridIdx){

	for (int i = 0; i != vKnownGridIdx.size(); ++i) 
		MapKnownStatus[vKnownGridIdx[i]] = true;

}

//Compute Euclidean distance
float Confidence::ComputeEuclideanDis(pcl::PointXYZ & oQueryPoint, pcl::PointXYZ & oTargetPoint) {

	//compute the eucliden distance between the query point (robot) and target point (scanned point)
	return sqrt(pow(oQueryPoint.x - oTargetPoint.x, 2.0f)
		      + pow(oQueryPoint.y - oTargetPoint.y, 2.0f)
		      + pow(oQueryPoint.z - oTargetPoint.z, 2.0f));

}

//Compute the distance term
std::vector<float> Confidence::DistanceTerm(std::vector<CofidenceValue> & vReWardMap,
	                                               const pcl::PointXYZ & oRobotPoint,
	                                         const std::vector<int> & vNeighborGrids,
	                                                const PCLCloudXYZ & vTravelCloud,
	                          const std::vector<std::vector<int>> & vGridTravelPsIdx){

	//define output
	std::vector<float> vDisRes;
	vDisRes.resize(vGridTravelPsIdx.size(),0.0);

	//intermediate variables
	std::vector<float> vDisPartValue;///<distance weight part 
	vDisPartValue.resize(vGridTravelPsIdx.size(),0.0);

	std::vector<float> vCenterPartValue;///<center weight part
	vCenterPartValue.resize(vGridTravelPsIdx.size(),0.0);

	std::vector<pcl::PointXYZ> vCenterPoints;///<center points of each neighboring grids
	vCenterPoints.reserve(vNeighborGrids.size());

	//compute the center point 
	for (int i = 0; i != vNeighborGrids.size(); ++i) {

		pcl::PointXYZ oOneCenter = ComputeCenter(vTravelCloud, vGridTravelPsIdx[vNeighborGrids[i]]);
		vCenterPoints.push_back(oOneCenter);
		if (!vGridTravelPsIdx[vNeighborGrids[i]].size())
			oOneCenterGaussianKernel(oQueryPo, oOneCenter, sigma);
	}

	
	//output
	return vDisRes;

}


////Compute the distance
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



void Confidence::FrontierTerm(std::vector<CofidenceValue> & vReWardMap, const int & iQueryGrid, const std::vector<int> & vNeighborGrids){

	float fBoundaryRes = 0.0;
	int iUnkownCount = 0;

	if (vReWardMap[iQueryGrid].iLabel == 2) {

		for (int k = 0; k != vNeighborGrids.size(); ++k) {

			if (!MapKnownStatus[vNeighborGrids[k]]) {
				iUnkownCount++;
			}

		}//end for k

	}//end if vReWardMap

	//if it is in boundary 
	if (!iUnkownCount)
		fBoundaryRes = 1.0;

	vReWardMap[iQueryGrid].boundary = fBoundaryRes;

}





//Compute 
std::vector<float> Confidence::OcclusionTerm(PCLCloudXYZ & vTravelCloud, pcl::PointXYZ & oRobotPoint){


}




void Confidence::ComputeTotalCoffidence(std::vector<CofidenceValue> & vReWardMap, const int & iQueryGrid){

	vReWardMap[iQueryGrid].totalValue = vReWardMap[iQueryGrid].boundary * vReWardMap[iQueryGrid].visibility;

}





void Confidence::Normalization(std::vector<float> & vFeatures){

	//
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

