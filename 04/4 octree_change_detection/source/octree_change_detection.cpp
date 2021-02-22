#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char** argv)
{
    srand(static_cast<unsigned>(time(nullptr)));

    // 八叉树分辨率，即体素的大小
    const auto resolution = 32.0;

    // 初始化空间变化检测对象
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);

    // 为cloudA点云填充数据
    cloudA->width = 128;  // 设置点云cloudA点数
    cloudA->height = 1;   // 设置点云cloudA为无序点云
    cloudA->points.resize(cloudA->width * cloudA->height);
    for (size_t i = 0; i < cloudA->points.size(); ++i)
    {
        cloudA->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudA->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }
    // 添加点云到八叉树，构建八叉树
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();

    // 交换八叉树缓存，但是cloudA对应的八叉树结构仍在内存中
    octree.switchBuffers();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

    // 为cloudB创建点云
    cloudB->width = 128;
    cloudB->height = 1;
    cloudB->points.resize(cloudB->width * cloudB->height);
    for (size_t i = 0; i < cloudB->points.size(); ++i)
    {
        cloudB->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        cloudB->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    //添加cloudB到八叉树
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    std::vector<int>newPointIdxVector;
    // 获取前一cloudA对应的八叉树在cloudB对应八叉树中没有的点集
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // 输出
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i] << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " " << cloudB->points[newPointIdxVector[i]].y << " "<< cloudB->points[newPointIdxVector[i]].z << std::endl;
}