#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char** argv)
{
    srand(static_cast<unsigned>(time(nullptr)));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建点云数据
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    const auto resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // 体素内近邻搜索
    std::vector<int>pointIdxVec;
    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")" << std::endl;
        for (size_t i = 0; i < pointIdxVec.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxVec[i]].x << " " << cloud->points[pointIdxVec[i]].y << " " << cloud->points[pointIdxVec[i]].z << std::endl;
    }

    //K近邻搜索
    const auto k = 10;
    std::vector<int> point_idx_nkn_search;
    std::vector<float> point_nkn_squared_distance;
    std::cout << "K nearest neighbor search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ") with K=" << k << std::endl;

    if (octree.nearestKSearch(searchPoint, k, point_idx_nkn_search, point_nkn_squared_distance) > 0)
    {
        for (size_t i = 0; i < point_idx_nkn_search.size(); ++i)
            std::cout << "    " << cloud->points[point_idx_nkn_search[i]].x << " " << cloud->points[point_idx_nkn_search[i]].y << " " << cloud->points[point_idx_nkn_search[i]].z << " (squared distance: " << point_nkn_squared_distance[i] << ")" << std::endl;
    }

    //半径内近邻搜索
    std::vector<int>pointIdxRadiusSearch;
    std::vector<float>pointRadiusSquaredDistance;
    const auto radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    std::cout << "Neighbors within radius search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x << " " << cloud->points[pointIdxRadiusSearch[i]].y << " " << cloud->points[pointIdxRadiusSearch[i]].z << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
}
