#ifndef GRIDMAP_H 
#define GRIDMAP_H 
#include <cmath>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#define ROBOT_AFFECTDIS 8.00
//*****************************
//
//confidence Value
//
//*****************************
struct CofidenceValue {

	//visibility based term
	float visibility;

	//boundary based term
	//float boundary;

	//distance based term
	float disTermVal;

	//quality
	float quality;

	//Weighted total of those two terms above
	float totalValue;

	//labels of obstacle, travelable region and boundary
	//0 nothing 
	//1 obstacles
	//2 ground points
	//3 boundary
	unsigned int iLabel;

	//minimum flag
	bool bMinFlag;

	//
	bool bKnownFlag;

	//travelable or not (can the robot reaches this grid right now)
	//-1 indicates it is an unknown grid
	//0 indicates this grid is ground but not reachable now
    //1 indicates this grid is a travelable region grid
	//2 is the new scanned grids (input) without growing
	//3 indicates the grid has been computed
	//4 indicates this grid is a off groud grid (not reachable forever)
	short travelable;

	bool Hausdorffflag;

	//constructor
	CofidenceValue() {

		visibility = 1.0;
		disTermVal = 0.0;
		//boundary = 0.0;
		totalValue = 0.0;
		quality = 0.0;
		travelable = -1;
		iLabel = 0;
		bMinFlag = false;
		bKnownFlag = false;
		Hausdorffflag = true;
	};

};


struct NormalVector2D {

	//two dimension space 
	float x;//in x label
	float y;//in y label
};
//*****************************
//
//GridSearchMask Value
//
//*****************************
struct GridSearchMask{
	int ix;
	int iy;
};

//*****************************
//
//GridMap Value
//
//*****************************
class GridMap {

	typedef pcl::PointCloud<pcl::PointXYZ> PCLCloudXYZ;
	typedef pcl::PointCloud<pcl::PointXYZI> PCLCloudXYZI;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloudXYZPtr;
	typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLCloudXYZIPtr;

public:

	//constructor
	GridMap(float f_fGridSize = 0.5, 
		    float f_fMapTotalLength = 500.0,
		    float f_fRadius = 5.0,
		    float f_fMinThreshold = 0.2);

	//destructor
	~GridMap();
	void SetMapSize(float f_fGridSize = 0.5, float f_fMapTotalLength = 500.0);

	void SetSuppressionThrs(float f_fRadius = 5.0, float f_fMinThreshold = 0.4);

	void InitializeMap();

	void GenerateMap(std::vector<std::vector<int>> & vGridPointIdx);

	void RegionGrow(const std::vector<int> & vNewScannedGrids);

	//generate search mask
	void GenerateSearchMask(const float & fRadiusGridsNum);

	//
	int AssignPointToMap(const pcl::PointXYZ & oQueryPoint);

	//assign points to map
	void AssignPointsToMap(const PCLCloudXYZ & vCloud, 
		                   std::vector<std::vector<int>> & vGridPointIdx, 
		                   unsigned int iLabel);

	//sampling point clouds
	void SamplingCloudUsingMap(int & numSampling);

	//divide a index into sx and sy, respectively 
	void IntoXYSeries(int & iQuerySX,
		              int & iQuerySY,
		              const int & iQueryGridIdx);

	//non-minimum suppression
	std::vector<int> NonMinimumSuppression();

	//searching based on the query grid
	std::vector<int> SearchGrids(const int & iQueryGridIdx, 
		                         const float & fRadiusGridsNum);

	//the scanning map in 2D plane
	std::vector<CofidenceValue> m_vReWardMap;

	//the normal vector in each pixel
	std::vector<NormalVector2D> m_vNormalVecMap;

private:

	inline bool JudgeRefreshMask(const float & fRadiusGridsNum);

	//compute the index based on the given x, y series number 
	inline int ComputeGridIdx(const int & iSX, 
		                      const int & iSY);

	//size of each pixel in map
	float m_fGridSize;
	
	//size of map (length of map to be scanned)
	float m_fMapTotalLength;

	//the number of grids in length or width
	int m_iGridNum;
	float m_fMinGridCorner;

	//search mask
	std::vector<GridSearchMask> m_vSearchMask;
	//a flag indicats that the mask should be rebuild or not
	//fMaskFlah records the last setting of 
	float m_fMaskFlag;

	//the threshold of total feature value to generate a node
	float m_fMinThreshold;
	float m_fMinRadiusNum; //searched radius

};


#endif
