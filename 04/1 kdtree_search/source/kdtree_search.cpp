#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char** argv)
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 设置参数
    cloud->width = 2000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // 填充点云
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    // 创建K-D Tree对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    // 向K-D Tree对象中填充点云
    kdtree.setInputCloud(cloud);

    // 创建搜索点
    pcl::PointXYZ search_point;
    search_point.x = rand() * 1024.0f / (RAND_MAX + 1.0f);
    search_point.y = rand() * 1024.0f / (RAND_MAX + 1.0f);
    search_point.z = rand() * 1024.0f / (RAND_MAX + 1.0f);

    // 邻域大小进行搜索
    const auto k = 10;
    std::vector<int> point_idx_nkn_search(k);
    std::vector<float> point_nkn_squared_distance(k);

    std::cout << "K nearest neighbor search at (" << search_point.x << " " << search_point.y << " " << search_point.z << ") with K=" << k << std::endl;
    
    if (kdtree.nearestKSearch(search_point, k, point_idx_nkn_search, point_nkn_squared_distance) > 0)
    {
        for (size_t i = 0; i < point_idx_nkn_search.size(); ++i)
            std::cout << "    " << cloud->points[point_idx_nkn_search[i]].x << " " << cloud->points[point_idx_nkn_search[i]].y << " " << cloud->points[point_idx_nkn_search[i]].z << " (squared distance: " << point_nkn_squared_distance[i] << ")" << std::endl;
    }

    // 邻域半径进行搜索
    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;
    const auto radius = rand() * 256.0 / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << search_point.x << " " << search_point.y << " " << search_point.z << ") with radius=" << radius << std::endl;

    const auto num = kdtree.radiusSearch(search_point, radius, point_idx_radius_search, point_radius_squared_distance);
    if (num > 0)
    {
        for (size_t i = 0; i < point_idx_radius_search.size(); ++i)
            std::cout << "    " << cloud->points[point_idx_radius_search[i]].x << " " << cloud->points[point_idx_radius_search[i]].y << " " << cloud->points[point_idx_radius_search[i]].z << " (squared distance: " << point_radius_squared_distance[i] << ")" << std::endl;
    }
    return 0;
}
