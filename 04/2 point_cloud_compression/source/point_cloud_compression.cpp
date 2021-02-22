#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>
#include <sstream>

#ifdef WIN32
# define SLEEP(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer() : viewer(" Point Cloud Compression Example")
    {
    }

void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
    if (!viewer.wasStopped())
    {
        // 存储压缩点云的字节流
        std::stringstream compressedData;
        // 输出点云
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
        // 压缩点云
        point_cloud_encoder->encodePointCloud(cloud, compressedData);
        // 解压缩点云
        point_cloud_decoder->decodePointCloud(compressedData, cloudOut);
        //可视化解压缩点云
        viewer.showCloud(cloudOut);
    }
}

    void run()
    {
        const auto show_statistics = true;

        // 压缩选项详见 /compression/compression_profiles.h
        const auto compression_profile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

        // 初始化压缩与解压缩对象
        point_cloud_encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compression_profile, show_statistics);
        point_cloud_decoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

        //创建从 OpenNI获取点云的对象
        pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

        //建立回调函数
        const boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this);

        auto c = interface->registerCallback(f);

        // 开始接收点云数据流
        interface->start();
        while (!viewer.wasStopped())
        {
            SLEEP(1);
        }

        interface->stop();

        // 删除点云压缩与解压缩对象实例
        delete (point_cloud_encoder);
        delete (point_cloud_decoder);
    }

    pcl::visualization::CloudViewer viewer;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* point_cloud_encoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* point_cloud_decoder;
};


int main(int argc, char** argv)
{
    SimpleOpenNIViewer v;
    v.run();
    return (0);
}
