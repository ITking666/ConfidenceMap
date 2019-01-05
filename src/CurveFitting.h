#ifndef CURVEFITTING_H
#define CURVEFITTING_H
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree.h>
#include <iostream>
#include <fstream>

class Spline {

public:

	Spline(float inSmoothValue = 0.6);
	~Spline();

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//B¨¦zier spline
	//see more details of the principle in www.antigrain.com/research/bezier_interpolation/index.html
	//Component Value of X, Y and Z coordinate
	float BezierComponentValue(float & p0, float & p1, float & p2, float & p3, float & t);
	//set smooth value
    void SetSmoothValue(float inSmoothValue);

	//generate a spline based on given control points
	//
	void BezierSplineGeneration(std::vector<std::vector<pcl::PointXYZ>> & splinePoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr & anchorPoints, float deltaT = 0.01, bool cycleFlag = false);

	//generate a non singularity Bezier Spline
	//
	void NonSingularityBezierSplineGeneration(std::vector<std::vector<pcl::PointXYZ>> & splinePoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr & anchorPoints, float deltaT = 0.01);

	//output the control points
	void ExtractNewControlPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr & realctrlPoints);

	//turn the two-dimensional vector variable splinePoints into a one-dimensional ordered vector variable
	//the element of output spline is non-repetitive
	void TransforContinuousSpline(std::vector<pcl::PointXYZ> & orderLineRes,
		std::vector<std::vector<pcl::PointXYZ>> & splinePoints);

	//sample the result by a given length
	void Sampling(std::vector<pcl::PointXYZ> & sampledSpline, std::vector<pcl::PointXYZ> & orderLineRes, 
		float samplingLength);
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Cubic B-Spline

	float CBSplineN03(float t);
	float CBSplineN13(float t);
	float CBSplineN23(float t);
	float CBSplineN33(float t);

	//Component Value of X, Y and Z coordinate
	float CBSplineComponentValue(float & p0, float & p1, float & p2, float & p3, float & t);

	//generate a spline based on given control points
	void CBSplineGeneration(std::vector<std::vector<pcl::PointXYZ>> & splinePoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr & controlPoints, float deltaT = 0.01);

	void Clear();

private:

	//the control points actully used in the B¨¦zier curve
	std::vector<pcl::PointXYZ> RealControlPoints;

	float smoothValue;

};

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//one example as belows shows how to use the Spline class:
//edited by Huang Pengdi 20171205
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//int main()
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr anchorPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr splinePoints(new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<int> classes;
//
//	Spline liner;
//
//	pcl::PointXYZ onePoint;
//	onePoint.x = 10; onePoint.y = 200; onePoint.z = 70;
//	anchorPoints->points.push_back(onePoint);
//	onePoint.x = 40; onePoint.y = 100; onePoint.z = 120;
//	anchorPoints->points.push_back(onePoint);
//	onePoint.x = 100; onePoint.y = 100; onePoint.z = 100;
//	anchorPoints->points.push_back(onePoint);
//	onePoint.x = 150; onePoint.y = 150; onePoint.z = 150;
//	anchorPoints->points.push_back(onePoint);
//	onePoint.x = 150; onePoint.y = 200; onePoint.z = 250;
//	anchorPoints->points.push_back(onePoint);
//	onePoint.x = 170; onePoint.y = 230; onePoint.z = 300;
//	anchorPoints->points.push_back(onePoint);
//	onePoint.x = 190; onePoint.y = 170; onePoint.z = 330;
//	anchorPoints->points.push_back(onePoint);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr diplayPoints(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr realctrlPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr clearctrlPoints(new pcl::PointCloud<pcl::PointXYZ>);
//	liner.BezierSplineGeneration(splinePoints, anchorPoints);
//	liner.ExtractNewControlPoints(realctrlPoints);
//
//	for (int i = 0; i != splinePoints->points.size(); ++i) {
//
//		diplayPoints->points.push_back(splinePoints->points[i]);
//		classes.push_back(0);
//
//	}
//
//	for (int i = 0; i != realctrlPoints->points.size(); ++i) {
//
//		diplayPoints->points.push_back(realctrlPoints->points[i]);
//		classes.push_back(1);
//
//	}
//
//	for (int i = 0; i != anchorPoints->points.size(); ++i) {
//
//		diplayPoints->points.push_back(anchorPoints->points[i]);
//		classes.push_back(2);
//
//	}
//
//	PointCloudDisplay Shower;
//
//	////**show class results**
//	Shower.GiveColorForPointClass(classes);
//	//Shower.RandomColorForPointClass(classes);
//	Shower.ShowPointCloudWithGivenColor(diplayPoints);
//
//	std::cin.get();
//	return 0;
//}
