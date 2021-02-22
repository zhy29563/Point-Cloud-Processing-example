#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    // 以当前时间设置随机数种子
    srand(time(0));

    // 定义输入点云与输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 设置点云宽度或数量，这里为数量
    cloud->width = 5;
    // 设置点云高度或标准，其为无序点云
    cloud->height = 1;
    cloud->points.resize(static_cast<size_t>(cloud->width * cloud->height));

    // 填充输入点云
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
        cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
        cloud->points[i].z = rand() / (RAND_MAX + 1.0f) - 0.5;
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    // 设置滤波器的输入点云
    pass.setInputCloud(cloud);
    // 设置执行过滤时的字段
    pass.setFilterFieldName("z");
    // 设置在过滤字段上的范围
    pass.setFilterLimits(0.0, 1.0);
    // 设置滤波后的点云是在指定的范围内/外。
    // 参数为true，保留指定范围之外的部分，参数为false，保留指定范围内的部分
    //pass.setFilterLimitsNegative (true);
    pass.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
        std::cerr << "    " << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << std::endl;
    return (0);
}