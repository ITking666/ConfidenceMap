#include"HPR.h"
/*=======================================

========================================*/
HPR::HPR(const pcl::PointXYZ & f_viewpoint,double f_param){
    SetParam(f_param);
	Inputviewpoint(f_viewpoint);
}
/*=======================================

========================================*/
void HPR::SetParam(double f_param){
	param=f_param;
}
/*=======================================

========================================*/
void HPR::Inputviewpoint(const pcl::PointXYZ & f_viewpoint){
	viewpoint=f_viewpoint;
}
/*=======================================

========================================*/
std::vector<int> HPR::Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr & vCloud){
	//所有点减去视点坐标
	//p=p-repmat(C,[numPtr 1])
	std::vector<double> onenorm(3,0.0);
	std::vector<double> normvector;

	pcl::PointCloud<pcl::PointXYZ>::Ptr convexcloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	convexcloud->width = vCloud->points.size();
    convexcloud->height = 1;
    convexcloud->is_dense = false;
    convexcloud->points.resize(convexcloud->width*convexcloud->height);

	for(size_t i=0;i!=vCloud->points.size();i++){
		convexcloud->points[i].x=vCloud->points[i].x-viewpoint.x;
		onenorm[0]=convexcloud->points[i].x;
		convexcloud->points[i].y=vCloud->points[i].y-viewpoint.y;
		onenorm[1]=convexcloud->points[i].y;
		convexcloud->points[i].z=vCloud->points[i].z-viewpoint.z;
		onenorm[2]=convexcloud->points[i].z;
		//计算每个点到视点的长度
		normvector.push_back(Normvector(onenorm,2));
	}
	//计算最远的距离为球形半径
	double radius=pow(10.0,param)*Getmax(normvector);
	//球形计算
	double numerator=0.0;
	for(size_t i=0;i!=convexcloud->points.size();i++){
	numerator=2*(radius-normvector[i]);
	convexcloud->points[i].x=convexcloud->points[i].x+numerator*convexcloud->points[i].x/normvector[i];
	convexcloud->points[i].y=convexcloud->points[i].y+numerator*convexcloud->points[i].y/normvector[i];
	convexcloud->points[i].z=convexcloud->points[i].z+numerator*convexcloud->points[i].z/normvector[i];
	}
	//来吧，凸包，你到底是什么东西
	pcl::PointXYZ onepoint;
	onepoint.x = 0.0;
	onepoint.y = 0.0;
	onepoint.z = 0.0;//放入
	convexcloud->points.push_back(onepoint);
	//转化为pcl格式，因为需要调用pcl库的ConvexHull对象

	//建立凸包类h的对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
	//建立存储数据
    pcl::ConvexHull<pcl::PointXYZ> chull;//凸包各顶点坐标
	std::vector<pcl::Vertices> vertindices;//凸包各个面的三个顶点索引，对应chull而非原点云！！！
	//倒腾点云进去
    chull.setInputCloud(convexcloud);
    //计算三维的
    chull.setDimension(3); 
	//吐血啊！返回值里只有点坐标和坐标直接的顶点连接关系，和原始点云的索引断了..断了....了
    chull.reconstruct(*hull,vertindices);//其实vertindices变量用不到
	//给凸包点云建立kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(convexcloud);
	std::vector<int> kdindices; 
	std::vector<float> kdistances;
	//主角登场了
	std::vector<int> occindices;
	//用kdtree寻找对应点
	for(size_t i=0;i!=hull->points.size();i++){
		kdindices.clear();
		kdistances.clear();
		//寻找的第一个点一般就是自身，非常一般
		kdtree.nearestKSearch(hull->points[i],1,kdindices,kdistances);
		//至少找一个点，不可能失败
		occindices.push_back(kdindices[0]);
	}
	//去除视点
	int viewnum=convexcloud->points.size()-1;//视点在最后一个啊
	Removevectormember(occindices,viewnum);

	return occindices;//突然想看指环王
}



/*===============================================

===============================================*/
double HPR::Normvector(std::vector<double> & nvect,int p){
	if(p<0)
	p=2;//违反格式强行置换
	double sum=0.0;
	//p范数
	for(size_t i=0;i!=nvect.size();i++){
	sum=pow(fabs(nvect[i]),double(p))+sum;
	}
	sum=pow(sum,1/double(p));
	return sum;
}

