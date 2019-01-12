#ifndef CONFIDENCE_H 
#define CONFIDENCE_H 
#include "HausdorffMeasure.h"
#include "GridMap.h"
#include "GHPR.h"
#include <stdlib.h>
#include <time.h> 


///************************************************************************///
// a class to implement the Gaussian Process Regression algorithm
// created and edited by Huang Pengdi

//Version 1.1 2018.11.25
// - add the 2D map funtion to organize point cloud storage
//Version 1.2 2018.12.13
// - change all calculation under the grid model
// - modify distance features
//Version 1.3 2018.12.14
// - add noting
// - complete distance term
///************************************************************************///
class Confidence {

	typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;
	typedef pcl::PointCloud<pcl::PointXYZI> PCLCloudXYZI;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloudXYZPtr;
	typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLCloudXYZIPtr;

public:

	//constructor
	Confidence(float f_fSigma,
		       float f_fGHPRParam = 4.2,
		       float f_fVisTermThr = 5);
	
	//destructor
	~Confidence();

	//set the affect radius of Gaussian Kernel
	void SetSigmaValue(const float & f_fSigma);

	//set visibility term related paramters
	void SetVisParas(const float & f_fGHPRParam, const float & f_fVisTermThr);

	//*******************Mathematical Formula Part********************
	
	inline float VectorInnerProduct(const pcl::PointXYZ & oAVec,
		                            const pcl::PointXYZ & oBVec);
	
	//Gaussian Kernel Function
	inline float GaussianKernel(const pcl::PointXYZ & oQueryPo,
		                       const pcl::PointXYZ & oTargerPo,
		                                         float & sigma);
	//linear Kernel Function
	inline float LinearKernel(const float & fTargetVal,
	                        	const float & fThrVal);

	//variance
	inline float StandardDeviation(const PCLCloudXYZ & vCloud);

	//density
	inline float ComputeDensity(const PCLCloudXYZ & vCloud,
								int iSampleTimes = 5,
								bool bKDFlag = true);


	//the 2 norm of a vector
	inline float Compute2Norm(const pcl::PointXYZ & oQueryPo,
		                     const pcl::PointXYZ & oTargerPo);

	//the 1 norm of a vector
	inline float ComputeSquareNorm(const pcl::PointXYZ & oQueryPo,
		                           const pcl::PointXYZ & oTargerPo);

	//compute center
	inline pcl::PointXYZ ComputeCenter(const PCLCloudXYZ & vCloud,
		                       const std::vector<int> & vPointIdx);
	inline pcl::PointXYZ ComputeCenter(const PCLCloudXYZ & vCloud);

	//get random value
	inline std::vector<int> GetRandom(const unsigned int iSize,
		                                 const int iSampleNums);
	static std::vector<int> GetRandom(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pAllTravelCloud,
		                                                                 GridMap & oMaper,
		                                                              const int iSampleNums);

	//Compute Euclidean distance
	static float ComputeEuclideanDis(pcl::PointXYZ & oQueryPoint, 
		                            pcl::PointXYZ & oTargetPoint);
	//*******************Feature Term Part********************
	//1. the distance term of confidence map
	//generate the distance feature map of a neighboring grids
	void DistanceTerm(std::vector<CofidenceValue> & vReWardMap, 
		                     const pcl::PointXYZ & oRobotPoint,
		               const std::vector<int> & vNeighborGrids,
		                      const PCLCloudXYZ & vTravelCloud,
		const std::vector<std::vector<int>> & vGridTravelPsIdx);

	void DisBoundTerm(std::vector<CofidenceValue> & vReWardMap,
		                                                 const pcl::PointXYZ & oRobotPoint,
		                                           const std::vector<int> & vNeighborGrids,
		                                                  const PCLCloudXYZ & vTravelCloud,
		                            const std::vector<std::vector<int>> & vGridTravelPsIdx,
		                                                   const PCLCloudXYZ & vBoundCloud,
		                             const std::vector<std::vector<int>> & vGridBoundPsIdx);
	//2. quality term of confidence map
	//void QualityTermUsingDensity(std::vector<CofidenceValue> & vReWardMap,
	//	                          const std::vector<int> & vNeighborGrids,
	//	                                 const PCLCloudXYZ & vTravelCloud,
	//               const std::vector<std::vector<int>> & vGridTravelPsIdx,
	//	                               const PCLCloudXYZ & vAllBoundCloud,
	//	            const std::vector<std::vector<int>> & vGridBoundPsIdx,
	//	                               const PCLCloudXYZ & vObstacleCloud,
	//	              const std::vector<std::vector<int>> & vGridObsPsIdx);

	//2. quality term of confidence map
	void QualityTerm(std::vector<CofidenceValue> & vReWardMap,
		                 const std::vector<int> & vNeighborGrids,
		                      const PCLCloudXYZ & vAllBoundCloud,
		   const std::vector<std::vector<int>> & vGridBoundPsIdx,
		                      const PCLCloudXYZ & vObstacleCloud,
		     const std::vector<std::vector<int>> & vGridObsPsIdx);

	//3. Compute the occlusion
	void OcclusionTerm(std::vector<CofidenceValue> & vReWardMap,
	                    const std::vector<int> & vNeighborGrids,
		  const std::vector<pcl::PointXYZ> & vHistoryViewPoints,
	                           const PCLCloudXYZ & vTravelCloud,
	     const std::vector<std::vector<int>> & vGridTravelPsIdx,
				             const PCLCloudXYZ & vAllBoundCloud,
		  const std::vector<std::vector<int>> & vGridBoundPsIdx,
	                         const PCLCloudXYZ & vObstacleCloud,
	        const std::vector<std::vector<int>> & vGridObsPsIdx);
	
	//4. Compute the boundary item
	//std::vector<float> BoundaryTerm(PCLCloudXYZ & vTravelCloud, PCLCloudXYZ & vBoundCloud, pcl::PointXYZ & oRobotPoint);
	
	//5. Compute the frontier item (constract method)
	//void FrontierTerm(std::vector<CofidenceValue> & vReWardMap, const int & iQueryGrid, const std::vector<int> & vNeighborGrids);

	//6. Compute the total coffidence value
	void ComputeTotalCoffidence(std::vector<CofidenceValue> & vReWardMap, 
		                         const std::vector<int> & vNeighborGrids);

	//normalization of features
	static void Normalization(std::vector<float> & vFeatures);

	//*******************Public Data Part********************
	pcl::PointXYZ oShowCenter;
	pcl::PointXYZ oShowRobot;

private:

	//sigma parameter of Gaussian function in GaussianKernel()
	float m_fSigma;
	
	//visibility term based paramters
	float m_fGHPRParam;///<parameter of GHPR algorithm
	float m_fVisTermThr;///<the threshold of visibility term

	//weighted of each term for total confidence value
	const float m_fWeightDis;
	const float m_fWeightVis;

	//weighted of each term for total confidence value
	const float m_fLenWeight;
	const float m_fBoundWeight;

	//the searched radius of a query point in density estimation
	const float m_fDensityR;

};

#endif