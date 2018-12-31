#include "HausdorffMeasure.h"
//Edited by Huang Pengdi 2018.12.30 

/*************************************************
Function: HausdorffDimension
Description: constrcution function for HausdorffDimension class
Calls: InitialLoopParams
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_iIterMax - the lower bound of iterations (calculated depth) allowed by default
       f_iIterMin - the upper bound of iterations (root) allowed by default
Output: initial all of member variables
Return: none
Others: none
*************************************************/
HausdorffDimension::HausdorffDimension(int f_iIterMax,
	                                   int f_iIterMin):
                                       m_fRedundancy(0.005){

	//initial the parameter 
	InitialLoopParams(f_iIterMax,f_iIterMin);
	
	//the default calculation method is to measure the occupancy rate.
	m_fParaQ = 0.0;
	
	//flag related
	m_bEdgeFlag = false;///<boundary length

	m_bMaxMinCoFlag = false;///<bounding box corner

	m_bScaleFlag = false;///<measuring scale

	m_bMinDisFlag = false;///<minimum measuring scale
	
}

/*************************************************
Function: ~HausdorffDimension
Description: destrcution function for HausdorffDimension class
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: none
Output: none
Return: none
Others: none
*************************************************/
HausdorffDimension::~HausdorffDimension(){

}

/*************************************************
Function: SetMinDis
Description: get the minimum measuring scale (meter)
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_fMinDis - the minimum measuring scale in meter
Output: m_fMinDis which records the given minimum measuring scale
Return: none
Others: none
*************************************************/
void HausdorffDimension::SetMinDis(float f_fMinDis){

	m_bMinDisFlag = true;

	m_fMinDis = f_fMinDis;
}

/*************************************************
Function: SetMaxMinCoor
Description: give a priori corner value of bounding box
Calls: none
Called By: class object
Table Accessed: none
Table Updated: none
Input: f_oMaxCoor - a value that saves the maximum value at each axis of point set
       f_oMinCoor - a value that saves the minimum value at each axis of point set
Output: m_oMinCoor, m_oMaxCoor 
Return: none
Others: none
*************************************************/
void HausdorffDimension::SetMaxMinCoor(const pcl::PointXYZ & f_oMaxCoor, 
		                               const pcl::PointXYZ & f_oMinCoor){

	//true if call this function
	m_bMaxMinCoFlag = true;

	//change the value if the input order is incorrect
	//any component value of oMaxCoor is larger than oMinCoor
	if(f_oMaxCoor.x < f_oMinCoor.x){

	   m_oMinCoor = f_oMaxCoor;
	   m_oMaxCoor = f_oMinCoor;

	}else{

	   m_oMinCoor = f_oMinCoor;
	   m_oMaxCoor = f_oMaxCoor; 

	}

}
/*========================
函数SetGivenScale()简介
功能：设置半径用于按点邻域来判断分形结果
searchtype=1 为3D方形
searchtype=2 为3D圆形
searchtype=3 计算整个点云的分形维数（一般测试用）
形参：输入的半径大小单位米
========================*/
void HausdorffDimension::SetGivenScales(const float & f_fLargeScale,
	                                    const float & f_fSmallScale){

	//assigment
	m_fLargeScale = f_fLargeScale;
	m_fSmallScale = f_fSmallScale;

	//turn flag
	m_bScaleFlag = true;
}

/*========================
函数Setq()
功能：设置计算的维数类型
一般就选0和2，注意不能选1
注意：1是信息维数，但是该程序没有编，分母为零会出错
========================*/
void HausdorffDimension::SetParaQ(int f_iParaQ){

	//get the Q parameter
	m_fParaQ = float(f_iParaQ);
	
	// q=1 would cause dividing zero
	if (m_fParaQ == 1){
		std::cout<<"Error:信息维数q=1在该程序中没有准备，建议选0或2！"<<std::endl;
	    std::cin.get();
		exit(0);
	}

}


/*************************************************
Function: LinearKernel
Description: the linear Kernel function of GHPR algorithm
Calls: none
Called By: ComputeVisibility
Table Accessed: none
Table Updated: none
Input: fGamma - the gamma parameter of kernel function
       fPointNorm - the point norm (distance)
Output: the distance value indicates the transformation 
Return: a distance value
Others: gamma is larger than the maximum distance from viewpoint to point set
*************************************************/
bool HausdorffDimension::ExtractEdgeLength(pcl::PointXYZ & oOutLength){

	//clear and reset the output
	oOutLength.x = 0.0;
	oOutLength.y = 0.0;
	oOutLength.z = 0.0;

	//if the boundary length has been computed
	if(m_bEdgeFlag){
		
		oOutLength.x = m_oEdgeLength.x;
		oOutLength.y = m_oEdgeLength.y;
		oOutLength.z = m_oEdgeLength.z;
		return true;
	
	}else{
		
		return false;

	}//end else

}


/*************************************************
Function: LinearKernel
Description: the linear Kernel function of GHPR algorithm
Calls: none
Called By: ComputeVisibility
Table Accessed: none
Table Updated: none
Input: fGamma - the gamma parameter of kernel function
       fPointNorm - the point norm (distance)
Output: the distance value indicates the transformation 
Return: a distance value
Others: gamma is larger than the maximum distance from viewpoint to point set
*************************************************/
void HausdorffDimension::InitialLoopParams(const int & f_iIterMax,
	                                       const int & f_iIterMin){

	m_iIterMax = f_iIterMax;
	m_iIterMin = f_iIterMin;

	if(m_iIterMax < 0 || m_iIterMin < 0 || m_iIterMax - m_iIterMin < 0){
		std::cout<<"Error:维数的尺度不能小于0，否则即尺寸无穷大，程序在该尺度下计算无意义。"<<std::endl;
		std::cout<<"请按回车确定结束程序重新选择尺度初始化。"<<std::endl;
		std::cin.get();
		exit(0);
	}

}
/*========================
函数BoxCounting()简介
功能：求输入三维点云的计盒分形维数
cloud 是点云数据
cellmax 方格子的最大边长,可以取2的偶数次幂次(1,2,4,8...),取大于数据长度的偶数
dim 是cloud的计盒维数,D=lim(log(N(e))/log(k/e))
返回计盒分形维数结果值
========================*/
float HausdorffDimension::BoxCounting(const pcl::PointCloud<pcl::PointXYZ> & vCloud){

	//define output as zero (the truth Hausdorff dimension of any point set)
	float fDimensionRes = 0.0;

	//a vector to save the result of scale
	std::vector<HitsInScale> vSpectrum;

	//define the bounding box as a cube with the largest size
	std::vector<float> vBoundLengths = GetBoundingLength(vCloud);
	
	//get the largest size of bounding box
	m_fBoundBoxLen = FindMaximum(vBoundLengths);

	//尺度模式则将尺度进行计算
	//+m_fBoundBoxLen/200因为边缘精度不够容易内存跳错，即多出一个
	m_fBoundBoxLen = m_fBoundBoxLen * (1.0f + m_fRedundancy);

	if(m_bScaleFlag){
		
		m_iIterMin = Round(log10(m_fLargeScale/m_fBoundBoxLen)/log10(0.5));
		
		//防止盒子边长小于给定的收敛边长而无法计算收敛，小于的话维数肯定是0的
	    if(m_fBoundBoxLen == 0 || m_fBoundBoxLen < m_fSmallScale)
			m_fBoundBoxLen = m_fSmallScale;
		
		m_iIterMax = Round(log10(m_fSmallScale/m_fBoundBoxLen)/log10(0.5));
	
	}
	
	//whether using computed minimum scale
	if(m_bMinDisFlag){
		
		if(m_fBoundBoxLen == 0 || m_fBoundBoxLen < m_fMinDis)//防止零无法整除
			m_fBoundBoxLen = m_fMinDis;//可以小于，但如果小好几个数量级，那要算非常久，但都是零无意义
		//Base change formula is to compute the loop
		m_iIterMax = floor(log10(m_fMinDis/m_fBoundBoxLen)/log10(0.5));
		
	}

	//if the bounding box is still be zero
	//this situation indicates that the input point set is empty or has only one point
	//therefore, a zero measured result would be output 
	if(!m_fBoundBoxLen)
		return fDimensionRes;
    
	//*****************major part******************
	float boxsize = m_fBoundBoxLen / pow(2.0f,float(m_iIterMax));
	float boxnumber = 0;
	//盒子总数目,因为盒子数等于尺寸分辨率
    int xnumber = (int)(pow(2.0f,float(m_iIterMax)));
	//建立三维数组来存储这些盒子的值
	std::vector<std::vector<std::vector<int>>> downbox;//基盒子
	std::vector<std::vector<std::vector<int>>> upbox;//Octree用的盒子

	for(int i = 0; i != xnumber; ++i){

		std::vector<std::vector<int>> boxy;
		for(int j = 0; j != xnumber; ++j){
			
			std::vector<int> boxz(xnumber,0);//全部初始化为0
			boxy.push_back(boxz);
		}
		downbox.push_back(boxy);
	} 

	//包含点的盒子数目
	int xvalue,yvalue,zvalue;
	for(int i = 0; i != vCloud.size(); ++i){

		xvalue = floor((vCloud[i].x - m_oMinCoor.x) / boxsize);
		yvalue = floor((vCloud[i].y - m_oMinCoor.y) / boxsize);
		zvalue = floor((vCloud[i].z - m_oMinCoor.z) / boxsize);
		downbox[xvalue][yvalue][zvalue]++;

	}

	//用广义维数占用率计数
	for(int i = 0; i != xnumber; ++i){
		for(int j = 0; j != xnumber; ++j){
			for(int k = 0; k != xnumber; ++k){

				if(downbox[i][j][k])
					boxnumber=boxnumber + pow(float(downbox[i][j][k]),m_fParaQ);
			}
		}
	}

	//存储第一对结果，其他结果用循环做
	HitsInScale oFirstScaleRes;
	oFirstScaleRes.boxNum = log(float(boxnumber));
	oFirstScaleRes.boxScale = log(boxsize);
	vSpectrum.push_back(oFirstScaleRes);

	std::cout << "n:" << m_iIterMax << "sca: " << boxsize << ",num: " << boxnumber << std::endl;
	//
	int iters = m_iIterMax - 1;
	//循环获得不同尺寸的计盒结果,n是最小尺度，盒子是越来越小的

	//
	while(iters - m_iIterMin + 1){
		
		//
		boxnumber = 0;
		boxsize = boxsize * 2.0;
		//用octree来遍历，减少计算
		//给新的upbox设置大小并赋值0；
		upbox.clear();
		int ocnumber = (int)(pow(2.0f,float(iters)));
		
		//
		if(ocnumber < 1)
			ocnumber = 1;
		
		//
		for(int i = 0;i != ocnumber; ++i){
			
			std::vector<std::vector<int>> boxy;
			
			for(int j = 0;j != ocnumber; ++j){
				std::vector<int> boxz(ocnumber,0);//全部初始化为0
				boxy.push_back(boxz);
			}
			
			upbox.push_back(boxy);
		} 
		
		//遍历方式，比较下级的点位置并分配到octree上级
		for(int i = 0;i != downbox.size(); ++i){
			
			for(int j = 0;j != downbox.size(); ++j){
				
				for(int k = 0; k != downbox.size(); ++k){

					if(downbox[i][j][k])
						//底下的盒子合并，有点的往上累加
						upbox[floor(float(i)/2)][floor(float(j)/2)][floor(float(k)/2)]
					     = upbox[floor(float(i)/2)][floor(float(j)/2)][floor(float(k)/2)] + downbox[i][j][k];
				}//end k
			}//end j
		}//end i
		
		//计算盒子数
		
		for(int i = 0;i != upbox.size(); ++i){
			
			for(int j = 0;j != upbox.size(); ++j){
				
				for(int k = 0;k != upbox.size(); ++k){
					
					if(upbox[i][j][k])
						boxnumber = boxnumber + pow(float(upbox[i][j][k]),m_fParaQ);
				}
			}
		}
		
		//get the log value
		HitsInScale oScaleRes;
		oScaleRes.boxNum = log(float(boxnumber));
		oScaleRes.boxScale = log(boxsize);
	
		std::cout << "n:" << iters << " ,sca: " << boxsize << " ,num: " << boxnumber << std::endl;
	
		vSpectrum.push_back(oScaleRes);
		
		//将下级box变成当前Box
		downbox.clear();
		downbox=upbox;

		iters--;
	}

    //fitting by using the least squares
	//q = 1 or q = 2
	//get the slope value of the fitted line
	fDimensionRes = LinearFitting(vSpectrum) / (m_fParaQ - 1.0);

	//output
	return fDimensionRes;

}
/*========================
LinearFitting函数简介
功能：最小二乘法线性回归
形参：一个装有点云盒子数和点云尺度的矩阵
输出：样本点的一次方程斜率即维数（注意是对数形式）
return double型拟合结果
========================*/
float HausdorffDimension::LinearFitting(std::vector<HitsInScale> & vSpectrum){
	
	//set y = kx + b as the target line to be fitted

	float iSpNum = float(vSpectrum.size());
	float fSumX = 0.0;
	float fSumY = 0.0;
	float fSumXY = 0.0;
	float fSumXX = 0.0;

    //Accumulate samples
	for (size_t i = 0; i < vSpectrum.size(); ++i){
		
		fSumX += vSpectrum[i].boxScale;//x is the scale value
		fSumY += vSpectrum[i].boxNum;//y is the non-empty box number
		fSumXY += vSpectrum[i].boxScale * vSpectrum[i].boxNum;
		fSumXX += vSpectrum[i].boxScale * vSpectrum[i].boxScale;
	}
	
	//prepare for k
	float fMeanX = fSumX / iSpNum;
	float fMeanY = fSumY / iSpNum;
	
	//compute the slope k
	//k = (n*sum(xy)-sum(x)sum(y))/(n*sum(x^2)-sum(x)^2)
	
	//compute the denominator to check whether the denominator is zero 
	float fKPara = fSumXX - iSpNum * fMeanX * fMeanX;
	
	//if denominator is non-zero 
	if(fKPara)
		fKPara = (fSumXY - iSpNum * fMeanX * fMeanY) / fKPara;
	else 
		fKPara = 0;
 
    return fKPara;

};

/*========================
GetBoundingLength函数简介
功能：计算包围盒的边长
形参：输入的点云
输出：保存到该类的m_oEdgeLength
========================*/
std::vector<float> HausdorffDimension::GetBoundingLength(const pcl::PointCloud<pcl::PointXYZ> & vCloud){

	//check whether the maximum and minimum coordinate value of points is known  
	//if they are unknown
    if(!m_bMaxMinCoFlag){
		
		//reset minimum and maximum of coordinate value at each axis 
		m_oMinCoor.x = FLT_MAX;
		m_oMinCoor.y = FLT_MAX;
		m_oMinCoor.z = FLT_MAX;
		
		m_oMaxCoor.x = -FLT_MAX;
		m_oMaxCoor.y = -FLT_MAX;
		m_oMaxCoor.z = -FLT_MAX;

		//record the coordinate ranges of input point set 
		for(int i = 0; i != vCloud.size(); ++i){
			
			if(vCloud[i].x < m_oMinCoor.x) 
				m_oMinCoor.x = vCloud[i].x;
			
			if(vCloud[i].y < m_oMinCoor.y) 
				m_oMinCoor.y = vCloud[i].y;
			
			if(vCloud[i].z < m_oMinCoor.z) 
				m_oMinCoor.z = vCloud[i].z;
			
			if(vCloud[i].x > m_oMaxCoor.x) 
				m_oMaxCoor.x = vCloud[i].x;
			
			if(vCloud[i].y > m_oMaxCoor.y) 
				m_oMaxCoor.y = vCloud[i].y;
			
			if(vCloud[i].z > m_oMaxCoor.z) 
				m_oMaxCoor.z = vCloud[i].z;
		
		}//end for i != vCloud.size()

	}//end if

	//compute the bounding box length at each axis
	m_oEdgeLength.x = m_oMaxCoor.x - m_oMinCoor.x;///<x
	m_oEdgeLength.y = m_oMaxCoor.y - m_oMinCoor.y;///<y
	m_oEdgeLength.z = m_oMaxCoor.z - m_oMinCoor.z;///<z

	//turn flag
	m_bEdgeFlag = true;

	//output 
	std::vector<float> vBoundLengths;
	vBoundLengths.push_back(m_oEdgeLength.x);
	vBoundLengths.push_back(m_oEdgeLength.y);
	vBoundLengths.push_back(m_oEdgeLength.z);
	return vBoundLengths;

}


/*========================
Clearscale函数简介
功能：清除已有scale，方便循环中变动赋值
========================*/
void HausdorffDimension::ClearLength(){
	
	//the parameters here is related to the input data
	//clear coordinate corner
	m_oMinCoor.x = FLT_MAX;
	m_oMinCoor.y = FLT_MAX;
	m_oMinCoor.z = FLT_MAX;	

    m_oMaxCoor.x = -FLT_MAX;
	m_oMaxCoor.y = -FLT_MAX;
	m_oMaxCoor.z = -FLT_MAX;

	//turn coordinate corner flag
	m_bMaxMinCoFlag = false;

	//clear edge length
	m_oEdgeLength.x = 0.0;
	m_oEdgeLength.y = 0.0;
	m_oEdgeLength.z = 0.0;
	m_fBoundBoxLen = 0.0;

	//turn edge flag
	m_bEdgeFlag = false;

}

void HausdorffDimension::ClearAll(){

	//clear coordinate corner value
	m_oMinCoor.x = FLT_MAX;
	m_oMinCoor.y = FLT_MAX;
	m_oMinCoor.z = FLT_MAX;	

    m_oMaxCoor.x = -FLT_MAX;
	m_oMaxCoor.y = -FLT_MAX;
	m_oMaxCoor.z = -FLT_MAX;
	//turn coordinate flag
	m_bMaxMinCoFlag = false;

	//clear boundary length value
	m_oEdgeLength.x = 0.0;
	m_oEdgeLength.y = 0.0;
	m_oEdgeLength.z = 0.0;

	//clear maximum length of those three above
	m_fBoundBoxLen = 0.0;
	
	//turn edge flag
	m_bEdgeFlag = false;

	//reset generalized dimensional parameter
	m_fParaQ = 0;

	//clear scale as nothing input
	m_fLargeScale = 0.0;
	m_fSmallScale = 0.0;
	m_bScaleFlag = false;

	//clear the minimum measure distance
	m_fMinDis = 0.0;
	m_bMinDisFlag = false;

}

