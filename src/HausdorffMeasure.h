#ifndef HAUSDORFFMEASURE_H 
#define HAUSDORFFMEASURE_H

#include<cmath>

#include<cfloat>

#include<pcl/point_types.h>

#include<pcl/point_cloud.h>


//a structe records the non-emptys voxels number and its corresponding measured scale
//scale is in meter
//this structure is mainly designed for linear fitting method
struct HitsInScale{
	//the value of current scale
	float boxScale;
	//thenon-empty boxes/voxels number 
	float boxNum;
};
///************************************************************************///
// a class to implement the GHPR algorithm
// GHPR - Generalized Hidden Point Removal operator
// Katz S., Tal A., On the visibility of point clouds, ICCV, 2015,1350-1358.
// created and edited by Huang Pengdi, 2018.11.03

//Version 1.0 2018.11.03
// - add the implementation of the HPR algorithm
//Version 1.1 2018.12.18
// - add the implementation of the GHPR algorithm

///************************************************************************///
/*========================================
方法类HausdorffDimension
该类为计算分形特征的类
采用的是3D计盒维数估计
方法主要是在局部点云建立反向octree
对格子数和尺度进行一阶线性拟合求出梯度
根据公式，梯度即为分形维数估计
可以实现豪斯多夫维数和相关维数的计算
edited by Huang Pengdi 2014.07.26
修改于11月中旬,12月
one example show how to use this class at the end of present page
=======================================*/
class HausdorffDimension{

public:

    //*************Initialization function*************
	//constructor
	HausdorffDimension(int f_iIterMax=3,
		               int f_iIterMin=1);

	//destructor
	~HausdorffDimension();
	//初始化函数，大多参数的默认值在此函数内设置（distinguishing based on repeatly setting during iteration or not）
	//**************************参数初始化相关函数***************************
	//****just set once一劳永逸（设置一次就行）
	void SetParaQ(int f_iParaQ);//设置广义维数的参数大小，一般选0-豪斯多夫维数或选2-相关维数


	void InitialLoopParams(const int & f_iIterMax,
	                       const int & f_iIterMin);//获取迭代的上限和下限

    //手动设置迭代的尺度大小,一般用于求取需求尺度下的值（如20到100m）
	void SetGivenScales(const float & f_fLargeScale,
	                    const float & f_fSmallScale);
    //two steps from hell
	//****need repeatly setting老黄牛般勤劳（在循环内需多次设置，主要是为了可以在循环内可以有不同值）
	//清除一切
	void ClearLength();
	//清除不需要的东西，例如循环过程中类内保存的点云，需要清理
	void ClearAll();
    //设置寻找向量无关模式

	//set the minimum distance
	void SetMinDis(float f_fMinDis);

    //设置已经计算好的点云包围盒最大最小值
	void SetMaxMinCoor(const pcl::PointXYZ & f_oMaxCoor, 
		               const pcl::PointXYZ & f_oMinCoor);

	//extract the edge length 
	bool ExtractEdgeLength(pcl::PointXYZ & oOutLength);

//*******************************performance function***************************
	//round
	float Round(const float & fIdx){
		
		return(fIdx > 0.0) ? floor(fIdx + 0.5) : ceil(fIdx - 0.5);
	
	};

	//Least squares linear fitting
    float LinearFitting(std::vector<HitsInScale> & vSpectrum);

	//compute the length of the bounding box
	std::vector<float> GetBoundingLength(const pcl::PointCloud<pcl::PointXYZ> & vCloud);

	template <typename T>
	inline T FindMaximum(std::vector<T> vSequence){
		
		//find the maximum among the input datas
		T tMaxValue;
		
		//inital the MaxValue as the first value of input
		if(vSequence.size())	
			tMaxValue = vSequence[0];
		
		//find the maximum value
		for(int i = 0; i != vSequence.size(); ++i){
			
			if(tMaxValue < vSequence[i])
			tMaxValue = vSequence[i];
		}//end for i
		
		return tMaxValue;
	}
	
	//the major function
	//this function is to implement the Hausdorff measure by using the box counting based method
	//the box counting is an approximate solution of Hausdorff measure
	float BoxCounting(const pcl::PointCloud<pcl::PointXYZ> & vCloud);

private:
	
	//迭代次数的头尾两值，决定分形所需要的拟合数据集大小
	int m_iIterMax;
	int m_iIterMin;

	//盒子维数的方形盒子大小
	float m_fBoundBoxLen;
	
	//盒子边长补全（为了防止边界因内存及字符型原因计算错误，加个极小值）
	const float m_fRedundancy;

	//bool readpointflag;
	//flag
	
	bool m_bMaxMinCoFlag;
	

	//this flag is to indicate the box counting operation has been done or not 
	bool m_bEdgeFlag;//measure the dimension




	//计算的包围盒各个方向边长
	pcl::PointXYZ m_oMinCoor;
	pcl::PointXYZ m_oMaxCoor;//
	pcl::PointXYZ m_oEdgeLength;//包围盒x,y,z长度
	
	//广义维数q
	float m_fParaQ;

	//设置尺度大小
	float m_fLargeScale;
	float m_fSmallScale;
	bool m_bScaleFlag;

	//最小距离
	float m_fMinDis;
	bool m_bMinDisFlag;

};

#endif

/*=================================================Example
//one example: you can copy it and run immediately but insure you have pcd or las file
//read point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<Point3D> point3d;
	HPDpointclouddataread("SurfaceXYZPCD.pcd",cloud,point3d,2);
//new object for computing
	HausdorffDimension pcfa;
	pcfa.Settype(3);
	//pcfa.Setradius(0.2);
	//pcfa.Setresolution(10,10,10);
//caculate
	pcfa.ComputeDimension(cloud,point3d,"asas.txt");
//show and pause
	std::cin.get();
==============================================*/