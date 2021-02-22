#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

using namespace std;

namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float operator () (const PointXYZ& p) const
        {
            return p.z;
        }
    };
}

int main(int argc, char* argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pig.pcd", *cloud_xyz);

    const float min_scale = 0.01f;
    const int n_octaves = 6;
    const int n_scales_per_octave = 4;
    const float min_contrast = 0.01f;

    // 创建sift关键点检测对象
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;

    sift.setInputCloud(cloud_xyz);//设置输入点云

    // 创建一个空的kd树对象tree，并把它传递给sift检测对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);

    // 指定搜索关键点的尺度范围
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    // 设置限制关键点检测的阈值
    sift.setMinimumContrast(min_contrast);
    // 执行sift关键点检测，保存结果在result
    sift.compute(result);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    // 将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据
    copyPointCloud(result, *cloud_temp);

    //可视化输入点云和关键点
    pcl::visualization::PCLVisualizer viewer("Sift keypoint");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_xyz, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");

    viewer.addPointCloud(cloud_temp, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}