#ifndef HPR_H
#define HPR_H
#include"LasOperator.h"
#include<pcl/point_types.h>
#include<pcl/surface/convex_hull.h>
#include<pcl/surface/concave_hull.h>//important
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree.h>
//黄鹏頔编写，使用前请征得本人同意
class HPR{
public:
	HPR(const pcl::PointXYZ & f_viewpoint, double f_param=2);
	//set param
	void SetParam(double f_param);

	void Inputviewpoint(const pcl::PointXYZ & f_viewpoint);

	static double Normvector(std::vector<double> & nvect,int p);

	std::vector<int> Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr & vCloud);

	template <typename T>
    T Getmax(std::vector<T> & vec){
	  //
	  T max;
	  max=vec[0];
	  for(size_t i=0;i!=vec.size();i++){

	  if(vec[i]>max)
	     max=vec[i];
	  }

	  return max;
    }

    template <typename T>
    void Removevectormember(std::vector<T>& v,int val){
         std::vector<T>::iterator ite=v.begin();
         while(ite!=v.end()){
         if(*ite==val)
            ite=v.erase(ite);
         else
            ++ite;
         }
    }
    //函数重载有indices的形式
    template <typename T>
    void Removevectormember(std::vector<T>& v,std::vector<bool> indices){
         std::vector<T>::iterator ite=v.begin();
	     int n=0;
         while(ite!=v.end()){
            if(!indices[n])
               ite=v.erase(ite);
            else
               ++ite;
		    n++;
        }
    }

private:
	double param;//param参数
	pcl::PointXYZ viewpoint;
};


#endif

//******************************one example*************************************
    //Point3D view={-465.36,-1013.55,4.63,0};
	//Point3D target={-447.58,-1025.84,11.14,0};
	//Trianseg intregionseg;
	////hehe
	//intregionseg.Setopandvpoint(view,target);
	//intregionseg.Generatetrian(10,15.0);
	//std::vector<int> indices=intregionseg.Getriacludepoint(point3d);
	////分割感兴趣视域
	//for(size_t i=0;i!=indices.size();i++)
	//	viewpoint3d.push_back(point3d[indices[i]]);
	////检测
	//std::vector<Point3D> cviewpoint3d=viewpoint3d;
	//HPR hpdhpr(view,4.0);
	//std::vector<int> occindices;
	//occindices=hpdhpr.Compute(cviewpoint3d);
	////输出
	//for(size_t i=0;i!=viewpoint3d.size();i++)
	//	viewpoint3d[i].classification=0;
	//for(size_t i=0;i!=occindices.size();i++){
	//	//occloud->push_back(cloud->points[occindices[i]]);
	//	viewpoint3d[occindices[i]].classification=1;
	//}
	//窗口开启