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
	//���е��ȥ�ӵ�����
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
		//����ÿ���㵽�ӵ�ĳ���
		normvector.push_back(Normvector(onenorm,2));
	}
	//������Զ�ľ���Ϊ���ΰ뾶
	double radius=pow(10.0,param)*Getmax(normvector);
	//���μ���
	double numerator=0.0;
	for(size_t i=0;i!=convexcloud->points.size();i++){
	numerator=2*(radius-normvector[i]);
	convexcloud->points[i].x=convexcloud->points[i].x+numerator*convexcloud->points[i].x/normvector[i];
	convexcloud->points[i].y=convexcloud->points[i].y+numerator*convexcloud->points[i].y/normvector[i];
	convexcloud->points[i].z=convexcloud->points[i].z+numerator*convexcloud->points[i].z/normvector[i];
	}
	//���ɣ�͹�����㵽����ʲô����
	pcl::PointXYZ onepoint;
	onepoint.x = 0.0;
	onepoint.y = 0.0;
	onepoint.z = 0.0;//����
	convexcloud->points.push_back(onepoint);
	//ת��Ϊpcl��ʽ����Ϊ��Ҫ����pcl���ConvexHull����

	//����͹����h�Ķ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
	//�����洢����
    pcl::ConvexHull<pcl::PointXYZ> chull;//͹������������
	std::vector<pcl::Vertices> vertindices;//͹�������������������������Ӧchull����ԭ���ƣ�����
	//���ڵ��ƽ�ȥ
    chull.setInputCloud(convexcloud);
    //������ά��
    chull.setDimension(3); 
	//��Ѫ��������ֵ��ֻ�е����������ֱ�ӵĶ������ӹ�ϵ����ԭʼ���Ƶ���������..����....��
    chull.reconstruct(*hull,vertindices);//��ʵvertindices�����ò���
	//��͹�����ƽ���kdtree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(convexcloud);
	std::vector<int> kdindices; 
	std::vector<float> kdistances;
	//���ǵǳ���
	std::vector<int> occindices;
	//��kdtreeѰ�Ҷ�Ӧ��
	for(size_t i=0;i!=hull->points.size();i++){
		kdindices.clear();
		kdistances.clear();
		//Ѱ�ҵĵ�һ����һ����������ǳ�һ��
		kdtree.nearestKSearch(hull->points[i],1,kdindices,kdistances);
		//������һ���㣬������ʧ��
		occindices.push_back(kdindices[0]);
	}
	//ȥ���ӵ�
	int viewnum=convexcloud->points.size()-1;//�ӵ������һ����
	Removevectormember(occindices,viewnum);

	return occindices;//ͻȻ�뿴ָ����
}



/*===============================================

===============================================*/
double HPR::Normvector(std::vector<double> & nvect,int p){
	if(p<0)
	p=2;//Υ����ʽǿ���û�
	double sum=0.0;
	//p����
	for(size_t i=0;i!=nvect.size();i++){
	sum=pow(fabs(nvect[i]),double(p))+sum;
	}
	sum=pow(sum,1/double(p));
	return sum;
}

