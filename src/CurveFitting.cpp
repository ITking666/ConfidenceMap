#include "CurveFitting.h"

//
//inSmoothValue 
Spline::Spline(float inSmoothValue){

	//set smooth value
	SetSmoothValue(inSmoothValue);

}

//
//nothing here 
Spline::~Spline() {

}

//Smooth Value of parallel tangents based Bezier Spline
//Smooth Value is to limit the action range of generated control points
//it thus changes the "flatness" of generated curve
void Spline::SetSmoothValue(float inSmoothValue) {

	// Resulting control points. Here smooth_value is mentioned
    // above coefficient K whose value should be in range [0...1].
    smoothValue = inSmoothValue;

}

//
//
float Spline::CBSplineN03(float t) {

	return ((-pow(t, 3) + 3 * pow(t, 2) - 3 * t + 1) / 6);

}

//
//
float Spline::CBSplineN13(float t) {

	return ((3 * pow(t, 3) - 6 * pow(t, 2) + 4) / 6);

}

//
//
float Spline::CBSplineN23(float t) {

	return ((-3 * pow(t, 3) + 3 * pow(t, 2) + 3 * t + 1) / 6);

}

//
//
float Spline::CBSplineN33(float t) {

	return (pow(t, 3) / 6);

}

//B-spline equation
//
float Spline::CBSplineComponentValue(float & p0, float & p1, float & p2, float & p3, float & t) {

	float cValue = CBSplineN03(t) * p0 + CBSplineN13(t) * p1 + CBSplineN23(t) * p2 + CBSplineN33(t) * p3;
	return cValue;

}

//generates a cubic B-spline
//the operation is similar to Bezier curve
//the fitted curve does not pass through the first and last given control point
void Spline::CBSplineGeneration(std::vector<std::vector<pcl::PointXYZ>> & splinePoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & controlPoints, float deltaT) {

	splinePoints.clear();
	//using partition fitting
	//the partition time - partNum
	// n control points generate n-3 cubic b-spline
	int partNum = controlPoints->points.size() - 3;

	for (int i = 0; i != partNum; ++i)    //draw partNum spline
	{
		std::vector<pcl::PointXYZ> oneSpline;
		//to each three control points (each spline)
		for (float t = 0.0; t <= 1.0; t = t + deltaT) // draw one spline
		{
			//compute the x, y, z value of each point in the b - spline
			pcl::PointXYZ onePoint;
			onePoint.x = CBSplineComponentValue(controlPoints->points[i].x, controlPoints->points[i + 1].x,
				controlPoints->points[i + 2].x, controlPoints->points[i + 3].x, t);
			onePoint.y = CBSplineComponentValue(controlPoints->points[i].y, controlPoints->points[i + 1].y,
				controlPoints->points[i + 2].y, controlPoints->points[i + 3].y, t);
			onePoint.z = CBSplineComponentValue(controlPoints->points[i].z, controlPoints->points[i + 1].z,
				controlPoints->points[i + 2].z, controlPoints->points[i + 3].z, t);
			//save results
			oneSpline.push_back(onePoint);

		}//for t

		splinePoints.push_back(oneSpline);

	}//for i

}

//the Bezier equation
//four control points generate one cubic Bezier curve
//
float Spline::BezierComponentValue(float & p0, float & p1, float & p2, float & p3, float & t) {

	//the Bezier function:
	//                 [-1  3 -3  1][p0]
	//                 |3  -6  3  0||p1|
	//                 |-3  3  0  0||p2|
	// [t^3, t^2, t, 1][1   0  0  0][p3]          1x4*4x4*4x1=1x1
	//

	float N0 = p0 * (1 - t) * (1 - t) * (1 - t) ;
	float N1 = 3.0 * p1 * t * (1 - t) * (1 - t);
	float N2 = 3.0 * p2 * t * t * (1 - t);
	float N3 = p3 * t * t * t;

	return N0 + N1 + N2 + N3;

}

//generates a Bezier Spline
//anchorPoints - the given control points
//delta T - the increment of T controlling the density of fitted curve point
//cycleFlag - if curve is closed loop (end to head)
void Spline::BezierSplineGeneration(std::vector<std::vector<pcl::PointXYZ>> & splinePoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & anchorPoints, float deltaT, bool cycleFlag) {

	splinePoints.clear();
	//a cruve connecting begin point to end point
	int cycValue = 1;
	if (cycleFlag)
		cycValue = 0;

	//pcl::PointXYZ onePoint;
	int rawControlPNum = anchorPoints->points.size();

	//compute the middle points and the distance of each two adjacent points
	pcl::PointCloud<pcl::PointXYZ>::Ptr midPoints(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> adjacentPointLens;

	for (int i = 0; i < rawControlPNum; i++) {

		//compute center point coordinate value of line between each two adjacent points
		pcl::PointXYZ onePoint;
		int nexti = (i + 1) % rawControlPNum;//to connect the first and last control points
											 //middle point of the each two neighboring points
		onePoint.x = (anchorPoints->points[i].x + anchorPoints->points[nexti].x) / 2.0;
		onePoint.y = (anchorPoints->points[i].y + anchorPoints->points[nexti].y) / 2.0;
		onePoint.z = (anchorPoints->points[i].z + anchorPoints->points[nexti].z) / 2.0;
		//save the middle point from lines 01, 12, 23.......
		midPoints->points.push_back(onePoint);

		//compute the distance between each two adjacent points
		float oneLength = sqrt(pow(anchorPoints->points[i].x - anchorPoints->points[nexti].x, 2.0) +
			pow(anchorPoints->points[i].y - anchorPoints->points[nexti].y, 2.0) +
			pow(anchorPoints->points[i].z - anchorPoints->points[nexti].z, 2.0));
		//save the distance value from lines 01, 12, 23.......
		adjacentPointLens.push_back(oneLength);

	}

	//save the generated control point
	std::vector<pcl::PointXYZ> backCtrlPoints;
	std::vector<pcl::PointXYZ> frontCtrlPoints;

	//to each raw anchor point, one new control point is either at right side and left side
	for (int i = 0; i < rawControlPNum; i++) {

		//easy to trace back to the last element 
		int backi = (i + rawControlPNum - 1) % rawControlPNum;

		//proportion K is to help determine the location base point 
		//a popular K is manually choosed as 0.5 
		float propK = adjacentPointLens[backi] / (adjacentPointLens[backi] + adjacentPointLens[i]);

		//compute the base point, which would be moved to the corresponding anchor point to obtain a offset value
		pcl::PointXYZ basePoint;
		basePoint.x = midPoints->points[backi].x + (midPoints->points[i].x - midPoints->points[backi].x) * propK;
		basePoint.y = midPoints->points[backi].y + (midPoints->points[i].y - midPoints->points[backi].y) * propK;
		basePoint.z = midPoints->points[backi].z + (midPoints->points[i].z - midPoints->points[backi].z) * propK;

		//middle point + offset vector + scale vector 
		//Pb + (Po - Pb) + (Pm - Pb)*scale = Po + (Pm - Pb)*scale
		//
		pcl::PointXYZ oneBackCtrlP;
		oneBackCtrlP.x = anchorPoints->points[i].x + (midPoints->points[backi].x - basePoint.x) * smoothValue;
		oneBackCtrlP.y = anchorPoints->points[i].y + (midPoints->points[backi].y - basePoint.y) * smoothValue;
		oneBackCtrlP.z = anchorPoints->points[i].z + (midPoints->points[backi].z - basePoint.z) * smoothValue;

		pcl::PointXYZ oneFrontCtrlP;
		oneFrontCtrlP.x = anchorPoints->points[i].x + (midPoints->points[i].x - basePoint.x) * smoothValue;
		oneFrontCtrlP.y = anchorPoints->points[i].y + (midPoints->points[i].y - basePoint.y) * smoothValue;
		oneFrontCtrlP.z = anchorPoints->points[i].z + (midPoints->points[i].z - basePoint.z) * smoothValue;

		backCtrlPoints.push_back(oneBackCtrlP);
		frontCtrlPoints.push_back(oneFrontCtrlP);
	}

	//draw curve
	for (int i = 0; i < rawControlPNum - cycValue; i++) {

		std::vector<pcl::PointXYZ> oneSpline;

		pcl::PointXYZ P0 = anchorPoints->points[i];//raw control p
		RealControlPoints.push_back(P0);

		//p1 and p2 are new control points
		pcl::PointXYZ P1 = frontCtrlPoints[i];//new control p
		RealControlPoints.push_back(P1);

		int nexti = (i + 1) % rawControlPNum;
		pcl::PointXYZ P2 = backCtrlPoints[nexti];//new control p
		RealControlPoints.push_back(P2);

		pcl::PointXYZ P3 = anchorPoints->points[nexti];//raw control p

		//to each three control points (each spline)
		float t = 0.0;
		float lastT = 0.0;//supervise the change of t value
		while (t <= 1.0) { // draw one spline{ // draw one spline

			//compute the x, y, z value of each point in the b - spline
			pcl::PointXYZ onePoint;
			onePoint.x = BezierComponentValue(P0.x, P1.x, P2.x, P3.x, t);
			onePoint.y = BezierComponentValue(P0.y, P1.y, P2.y, P3.y, t);
			onePoint.z = BezierComponentValue(P0.z, P1.z, P2.z, P3.z, t);
			//save results
			oneSpline.push_back(onePoint);

			//perpare for next cyclic
			lastT = t;
			t = t + deltaT;

		}//while t
		 //this judgement is to prevent the "Tail Break"
		 //which is caused by accuracy or aliquant problems, e.g., t=0.0;<=1.0;t=t+0.03;
		 //adds the last control point as the last spline point to maintain integrity  
		if (lastT < 1.0)
			oneSpline.push_back(P3);

		splinePoints.push_back(oneSpline);

	}//for i

	if (!cycleFlag)//save the end point of point set as the last control point
		RealControlPoints.push_back(anchorPoints->points[rawControlPNum - 1]);

}

//This is the main function to implement Bezier Spline fitting based on a seris of given ordered points
//splinePoints - the fitting curve points
//anchorPoints - the given control points
//delta T - the increment of T controlling the density of fitted curve point
//note that the fitted curve would pass through all of anchor points
//In other word, each given point is in the fitted line
void Spline::NonSingularityBezierSplineGeneration(std::vector<std::vector<pcl::PointXYZ>> & splinePoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr & anchorPoints, float deltaT) {

	splinePoints.clear();

	//pcl::PointXYZ onePoint;
	int rawControlPNum = anchorPoints->points.size();

	//compute the middle points and the distance of each two adjacent points
	pcl::PointCloud<pcl::PointXYZ>::Ptr midPoints(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> adjacentPointLens;

	for (int i = 0; i < rawControlPNum; i++) {

		//compute center point coordinate value of line between each two adjacent points
		pcl::PointXYZ onePoint;
		int nexti = (i + 1) % rawControlPNum;//to connect the first and last control points
											 //middle point of the each two neighboring points
		onePoint.x = (anchorPoints->points[i].x + anchorPoints->points[nexti].x) / 2.0;
		onePoint.y = (anchorPoints->points[i].y + anchorPoints->points[nexti].y) / 2.0;
		onePoint.z = (anchorPoints->points[i].z + anchorPoints->points[nexti].z) / 2.0;
		//save the middle point from lines 01, 12, 23.......
		midPoints->points.push_back(onePoint);

		//compute the distance between each two adjacent points
		float oneLength = sqrt(pow(anchorPoints->points[i].x - anchorPoints->points[nexti].x, 2.0) +
			pow(anchorPoints->points[i].y - anchorPoints->points[nexti].y, 2.0) +
			pow(anchorPoints->points[i].z - anchorPoints->points[nexti].z, 2.0));
		//save the distance value from lines 01, 12, 23.......
		adjacentPointLens.push_back(oneLength);

	}

	//save the generated control point
	std::vector<pcl::PointXYZ> backCtrlPoints;
	std::vector<pcl::PointXYZ> frontCtrlPoints;

	//to each raw anchor point, one new control point is either at right side and left side
	for (int i = 0; i < rawControlPNum; i++) {

		//easy to trace back to the last element 
		int backi = (i + rawControlPNum - 1) % rawControlPNum;

		//proportion K is to help determine the location base point 
		//a popular K is manually choosed as 0.5 
		float propK = adjacentPointLens[backi] / (adjacentPointLens[backi] + adjacentPointLens[i]);

		//compute the base point, which would be moved to the corresponding anchor point to obtain a offset value
		pcl::PointXYZ basePoint;
		basePoint.x = midPoints->points[backi].x + (midPoints->points[i].x - midPoints->points[backi].x) * propK;
		basePoint.y = midPoints->points[backi].y + (midPoints->points[i].y - midPoints->points[backi].y) * propK;
		basePoint.z = midPoints->points[backi].z + (midPoints->points[i].z - midPoints->points[backi].z) * propK;

		//middle point + offset vector + scale vector 
		//Pb + (Po - Pb) + (Pm - Pb)*scale = Po + (Pm - Pb)*scale
		//
		pcl::PointXYZ oneBackCtrlP;
		oneBackCtrlP.x = anchorPoints->points[i].x + (midPoints->points[backi].x - basePoint.x) * smoothValue;
		oneBackCtrlP.y = anchorPoints->points[i].y + (midPoints->points[backi].y - basePoint.y) * smoothValue;
		oneBackCtrlP.z = anchorPoints->points[i].z + (midPoints->points[backi].z - basePoint.z) * smoothValue;

		pcl::PointXYZ oneFrontCtrlP;
		oneFrontCtrlP.x = anchorPoints->points[i].x + (midPoints->points[i].x - basePoint.x) * smoothValue;
		oneFrontCtrlP.y = anchorPoints->points[i].y + (midPoints->points[i].y - basePoint.y) * smoothValue;
		oneFrontCtrlP.z = anchorPoints->points[i].z + (midPoints->points[i].z - basePoint.z) * smoothValue;

		backCtrlPoints.push_back(oneBackCtrlP);
		frontCtrlPoints.push_back(oneFrontCtrlP);
	}

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//Draw lines by a section way to avoid singularities of drawed curve
	//draw head of curve
	for (int i = 0; i != 1; ++i){
		
		std::vector<pcl::PointXYZ> oneSpline;

		pcl::PointXYZ P0 = anchorPoints->points[0];//raw control p0
		RealControlPoints.push_back(P0);

		//p1 and p2 are new control points
		pcl::PointXYZ P1 = anchorPoints->points[1];//raw control p1
		RealControlPoints.push_back(P1);

		int nexti = (i + 1) % rawControlPNum;
		pcl::PointXYZ P2 = backCtrlPoints[2];//new control p
		RealControlPoints.push_back(P2);

		pcl::PointXYZ P3 = anchorPoints->points[2];//raw control p2
	    
		float t = 0.0;
		float lastT = 0.0;//supervise the change of t value
		while(t <= 1.0) { // draw one spline

			 //std::cout << "t=" << t << std::endl;
			 //compute the x, y, z value of each point in the b - spline
		     pcl::PointXYZ onePoint;
		     onePoint.x = BezierComponentValue(P0.x, P1.x, P2.x, P3.x, t);
		     onePoint.y = BezierComponentValue(P0.y, P1.y, P2.y, P3.y, t);
		     onePoint.z = BezierComponentValue(P0.z, P1.z, P2.z, P3.z, t);
		     //save results
			 oneSpline.push_back(onePoint);
 
			 //perpare for next cyclic
			 lastT = t;
			 t = t + deltaT / 2.0;
			
	    }//while t
		//this judgement is to prevent the "Tail Break"
		//which is caused by accuracy or aliquant problems, e.g., t=0.0;<=1.0;t=t+0.03;
		//adds the last control point as the last spline point to maintain integrity  
		if(lastT < 1.0)
		   oneSpline.push_back(P3);
		//output
		splinePoints.push_back(oneSpline);

    }

	//++++++++++++++++++++++++++++++++++++++
	//draw middle of the curve
	for (int i = 2; i < rawControlPNum - 3; ++i) {

		std::vector<pcl::PointXYZ> oneSpline;

		pcl::PointXYZ P0 = anchorPoints->points[i];//raw control p
		RealControlPoints.push_back(P0);

		//p1 and p2 are new control points
		pcl::PointXYZ P1 = frontCtrlPoints[i];//new control p
		RealControlPoints.push_back(P1);

		int nexti = (i + 1) % rawControlPNum;
		pcl::PointXYZ P2 = backCtrlPoints[nexti];//new control p
		RealControlPoints.push_back(P2);

		pcl::PointXYZ P3 = anchorPoints->points[nexti];//raw control p

		//to each three control points (each spline)
		float t = 0.0;
		float lastT = 0.0;//supervise the change of t value
		while (t <= 1.0) { // draw one spline

		    //compute the x, y, z value of each point in the b - spline
			pcl::PointXYZ onePoint;
			onePoint.x = BezierComponentValue(P0.x, P1.x, P2.x, P3.x, t);
			onePoint.y = BezierComponentValue(P0.y, P1.y, P2.y, P3.y, t);
			onePoint.z = BezierComponentValue(P0.z, P1.z, P2.z, P3.z, t);
			//save results
			oneSpline.push_back(onePoint);

			//perpare for next cyclic
			lastT = t;
			t = t + deltaT;
		}//while t

		//prevents the "Tail Break", which has been detailed above  
		if (lastT < 1.0)
			oneSpline.push_back(P3);
		//output also
		splinePoints.push_back(oneSpline);

	}//for i

	//+++++++++++++++++++++++++++++++++++++++++++
	//draw end of the curve
	for (int i = 0; i != 1; ++i) {

		std::vector<pcl::PointXYZ> oneSpline;

		pcl::PointXYZ P0 = anchorPoints->points[rawControlPNum - 3];//raw control p
		RealControlPoints.push_back(P0);

		//p1 and p2 are new control points
		pcl::PointXYZ P1 = frontCtrlPoints[rawControlPNum - 3];//new control pn-2
		RealControlPoints.push_back(P1);

		int nexti = (i + 1) % rawControlPNum;
		pcl::PointXYZ P2 = anchorPoints->points[rawControlPNum - 2];//raw control pn-1
		RealControlPoints.push_back(P2);

		pcl::PointXYZ P3 = anchorPoints->points[rawControlPNum - 1];//raw control pn
		//obtains the last control point
		RealControlPoints.push_back(P3);

		float t = 0.0;
		float lastT = 0.0;//supervise the change of t value
		while (t <= 1.0) { // draw one spline

			//compute the x, y, z value of each point in the b - spline
			pcl::PointXYZ onePoint;
			onePoint.x = BezierComponentValue(P0.x, P1.x, P2.x, P3.x, t);
			onePoint.y = BezierComponentValue(P0.y, P1.y, P2.y, P3.y, t);
			onePoint.z = BezierComponentValue(P0.z, P1.z, P2.z, P3.z, t);
			//save results
			oneSpline.push_back(onePoint);

			//perpare for next cyclic
			lastT = t;
			t = t + deltaT / 2.0;

		}//for t

		 //prevents the "Tail Break", which has been detailed above  
		if (lastT < 1.0)
			oneSpline.push_back(P3);
		//output also
		splinePoints.push_back(oneSpline);

	}

}


//clear
//delete the generated control point set
void Spline::Clear() {

	RealControlPoints.clear();

}

//ExtractNewControlPoints
//extracts the control points which were generated during the processing 
//new control point set is different with the given control point set
void Spline::ExtractNewControlPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr & realctrlPoints) {

	for (int i = 0; i != RealControlPoints.size(); ++i) {
		realctrlPoints->points.push_back(RealControlPoints[i]);
	}

}

//Output a continuous Spline result
//orderLineRes - it is ordered and non-repeat
//orderLineRes - it is also the point set of fitted curve
//splinePoints - a spline result obtained by NonSingularityBezierSplineGeneration function
void Spline::TransforContinuousSpline(std::vector<pcl::PointXYZ> & orderLineRes,
	std::vector<std::vector<pcl::PointXYZ>> & splinePoints){

	//clear output
	orderLineRes.clear();
	
	//to each segment
	for (int i = 0; i != splinePoints.size(); ++i) {
		//to each point
		//foremost, removes the last element because it is the first element of next segment
	    for (int j = 0; j != splinePoints[i].size() - 1; ++j) {
			 //receiving
		     orderLineRes.push_back(splinePoints[i][j]);

	    }

    }

	//gains the last element of last segment which is the last given control point
	//the last segment
	int lineNum = splinePoints.size();
    //the last element index of the last segment
	int LastPointIdx = splinePoints[lineNum - 1].size() - 1;
	//receiving
	orderLineRes.push_back(splinePoints[lineNum - 1][LastPointIdx]);

}

//sampling with a given length - samplingLength
void Spline::Sampling(std::vector<pcl::PointXYZ> & sampledSpline, std::vector<pcl::PointXYZ> & orderLineRes,
	float samplingLength) {

	//construct a spline point clouds
	pcl::PointCloud <pcl::PointXYZ>::Ptr splineClouds(new pcl::PointCloud <pcl::PointXYZ>);
	for (int i = 0; i != orderLineRes.size(); ++i) {
	
		splineClouds->points.push_back(orderLineRes[i]);

	}

	//using a kdtree structure for spline point set
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(splineClouds);

    //indicates the 
	std::vector<bool> visitStatus(splineClouds->points.size(),true);
	//saves the current query points
	std::vector<int> queryPoints;
	queryPoints.push_back(0);
	//to each lane point
	while(queryPoints.size()) {
		
		float maxDis = -FLT_MAX;
		int maxPointIdx = -1;
		//intermediate variables
		std::vector<int> neighboringIdx;
		std::vector<float> pointsDis;

		//compute distance
		kdtree.radiusSearch(splineClouds->points[queryPoints[0]], samplingLength, neighboringIdx, pointsDis);
	
		//find the maximum value
		for (int i = 0; i != neighboringIdx.size(); ++i){
			//the point has not been visited
	        if(visitStatus[neighboringIdx[i]]){
				//if it is the maximun distance
				if (pointsDis[i] > maxDis)
					maxDis = pointsDis[i];
				    maxPointIdx = neighboringIdx[i];
			}
        }

	    //label searched points
		for (int i = 0; i != neighboringIdx.size(); ++i)
			visitStatus[neighboringIdx[i]] = false;

		queryPoints.pop_back();
		
		//generate a new query point
		if (maxPointIdx != -1){
			queryPoints.push_back(maxPointIdx);
			//output
			sampledSpline.push_back(splineClouds->points[maxPointIdx]);

			continue;
        }

		//if there is still a point far away from the searching region
		for (int i = 0; i != visitStatus.size(); ++i){
			//the point should have not been searched
		    if (visitStatus[i]) {	
 				queryPoints.push_back(i);
				//using this point as current query point
				break;
			}//if true
        }//for i

	}//while

}
