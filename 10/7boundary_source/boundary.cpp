#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/io/ply_io.h>


int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float re,float reforn)
{ 
	// 定义用于法线估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; 
	// 定义存储法线估计结果的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 
	// 设置输入点云
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
	// 设置法线估计的搜索半径。设置为分辨率的10倍时，效果较好，主要用于法线估计
	normEst.setRadiusSearch(reforn);
	// 执行法线估计
	normEst.compute(*normals);


	// 定义用于存储边界估计结果的对象
	pcl::PointCloud<pcl::Boundary> boundaries;
	// 定义用于边界特征估计的对象
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
	// 设置输入点云
	boundEst.setInputCloud(cloud);
	// 设置输入法线
	boundEst.setInputNormals(normals);
	// 设置搜索半径
	boundEst.setRadiusSearch(re);
	// 设置角度阈值
	boundEst.setAngleThreshold(M_PI/4);
	// 设置搜索方法
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 
	// 执行边界估计
	boundEst.compute(boundaries);


	// 存储边界点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>); 
	for(int i = 0; i < cloud->points.size(); i++)
	{ 
		if(boundaries[i].boundary_point > 0)
		{
			cloud_boundary->push_back(cloud->points[i]);
		}
	} 

	// 显示
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("边界估计"));
	
	int v1(0);
	MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor (0.3, 0.3, 0.3, v1);
	MView->addText ("Raw point clouds", 10, 10, "v1_text", v1); 

	int v2(0); 
	MView->createViewPort (0.5, 0.0, 1, 1.0, v2); 
	MView->setBackgroundColor (0.5, 0.5, 0.5, v2); 
	MView->addText ("Boudary point clouds", 10, 10, "v2_text", v2); 

	MView->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud",v1);
	MView->addPointCloud<pcl::PointXYZ> (cloud_boundary, "cloud_boundary",v2);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "sample cloud",v1);
	MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud_boundary",v2);
	MView->addCoordinateSystem (1.0);
	MView->initCameraParameters ();

	MView->spin();

	return 0; 
} 

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>);

	//Laden der PCD-Files 
	pcl::io::loadPCDFile ("1.pcd", *cloud_src);	

	estimateBorders(cloud_src,0.05f,0.05f);

	return 0;
}
