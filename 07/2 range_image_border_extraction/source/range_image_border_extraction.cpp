#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----参数-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

void printUsage(const char* progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
        << "Options:\n"
        << "-------------------------------------------\n"
        << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
        << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
        << "-m           Treat all unseen points to max range\n"
        << "-h           this help\n"
        << "\n\n";
}

int main(int argc, char** argv)
{
    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
        cout << "Setting angular resolution to " << angular_resolution << "deg.\n";

    angular_resolution = pcl::deg2rad(angular_resolution);

    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

    // 生成点云
    for (float x = -0.5f; x <= 0.5f; x += 0.01f)
    {
        for (float y = -0.5f; y <= 0.5f; y += 0.01f)
        {
            PointType point;
            point.x = x;
            point.y = y;
            point.z = 2.0f - y;
            point_cloud.points.push_back(point);
        }
    }
    point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;

    // -----------------------------------------------
    // -----从点云创建深度图像-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;

    // 从点云创建深度图像
    range_image.createFromPointCloud(point_cloud,
                                     angular_resolution,
                                     pcl::deg2rad(360.0f),
                                     pcl::deg2rad(180.0f),
                                     scene_sensor_pose,
                                     coordinate_frame,
                                     noise_level,
                                     min_range,
                                     border_size);

    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // --------------------------------------------
    // -----打开三维浏览器并添加点云-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
    viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");

    // -------------------------
    // -----提取边界-----
    // -------------------------
    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);

    // ----------------------------------
    // -----在三维浏览器中显示点集-----
    // ----------------------------------
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>), shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr, & veil_points = *veil_points_ptr, & shadow_points = *shadow_points_ptr;
    for (int y = 0; y < (int)range_image.height; ++y)
    {
        for (int x = 0; x < (int)range_image.width; ++x)
        {
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points.points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points.points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points.points.push_back(range_image.points[y * range_image.width + x]);
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
    viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

    //-------------------------------------
    // -----在深度图像中显示点集-----
    // ------------------------------------
    pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
    range_image_borders_widget =
        pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false, border_descriptions, "Range image with borders");

    while (!viewer.wasStopped())
    {
        range_image_borders_widget->spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}