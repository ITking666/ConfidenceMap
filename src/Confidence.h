#ifndef CONFIDENCE_H 
#define CONFIDENCE_H 
#include"GridMap.h"

//Version 1.1 2018.11.25
//add the 2D map funtion to organize point cloud storage
//Version 1.2 2018.12.13
//change all calculation under the grid model
//modify distance features



class Confidence {

	typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;
	typedef pcl::PointCloud<pcl::PointXYZI> PCLCloudXYZI;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloudXYZPtr;
	typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLCloudXYZIPtr;

public:

	//constructor
	Confidence(const int & iGridNum);
	
	//destructor
	~Confidence();

	void GetKnownGridIdx(const std::vector<int> & vKnownGridIdx);

	//*******************Mathematical Formula Part********************
	
	//Gaussian Kernel Function
	inline float GaussianKernel(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo, float & sigma);

	//the 2 norm of a vector
	inline float Compute2Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo);

	//the 1 norm of a vector
	inline float Compute1Norm(const pcl::PointXYZ & oQueryPo, const pcl::PointXYZ & oTargerPo);

	//Compute Euclidean distance
	static float ComputeEuclideanDis(pcl::PointXYZ & oQueryPoint, pcl::PointXYZ & oTargetPoint);

	//compute center
	inline pcl::PointXYZ ComputeCenter(const PCLCloudXYZ & vCloud, const std::vector<int> & vPointIdx);
	inline pcl::PointXYZ ComputeCenter(const PCLCloudXYZ & vCloud);
	//*******************Feature Term Part********************
	//1. the distance term of confidence map
	//generate the distance feature map of a neighboring grids
	std::vector<float> DistanceTerm(std::vector<CofidenceValue> & vReWardMap, 
		                            const pcl::PointXYZ & oRobotPoint,
		                            const std::vector<int> & vNeighborGrids,
		                            const PCLCloudXYZ & vTravelCloud,
		                            const std::vector<std::vector<int>> & vGridTravelPsIdx);

	//2. quality term of confidence map
	std::vector<float> QualityTerm(PCLCloudXYZ & vTravelCloud, pcl::PointXYZ & oRobotPoint);

	//3. Compute the occlusion
	std::vector<float> OcclusionTerm(PCLCloudXYZ & vTravelCloud, pcl::PointXYZ & oRobotPoint);

	//4. Compute the boundary item
	//std::vector<float> BoundaryTerm(PCLCloudXYZ & vTravelCloud, PCLCloudXYZ & vBoundCloud, pcl::PointXYZ & oRobotPoint);
	
	//5. Compute the total coffidence value
	void ComputeTotalCoffidence(std::vector<CofidenceValue> & vReWardMap, const int & iQueryGrid);
	
	//6. Compute the frontier item (constract method)
	void FrontierTerm(std::vector<CofidenceValue> & vReWardMap, const int & iQueryGrid, const std::vector<int> & vNeighborGrids);

	//normalization of features
	static void Normalization(std::vector<float> & vFeatures);

	//map indicates known grids and unknown grids
	std::vector<bool> MapKnownStatus;

private:


};

#endif