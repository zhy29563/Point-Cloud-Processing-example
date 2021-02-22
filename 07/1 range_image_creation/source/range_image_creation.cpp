#include <pcl/range_image/range_image.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    //生成数据
    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f)
        {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;

            pointCloud.points.push_back(point);
        }
    }

    pointCloud.width = (uint32_t)pointCloud.points.size();
    pointCloud.height = 1;

    // 以1度为角分辨率
    // 这意味着由临近的像素点所对应的每个光束之间相差1°
    float angularResolution = (float)(1.0f * (M_PI / 180.0f));

    // maxAngleWidth = 360与maxAngleHeight = 180意味着，我们进行模拟
    // 的距离传感器对周围的环境拥有一个完整的360°视角，用户在任何数据集
    // 下都可以使用此设置，因为最终获取的深度图像将被裁剪到有空间物体存
    // 在的区域范围。但是，用户可以通过减小数值来节省一些计算资源。
    // 水平视角
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));
    // 垂直视角
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));

    // 模拟了深度图像获取传感器的6自由度位置，其原始值为横滚角Roll、俯仰
    // 角Pitch、偏航角Yaw都为零。
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

    // coordinate_frame = pcl::RangeImage::CAMERA_FRAME
    // 说明系统的X轴式向右的，Y轴是向下的，Z轴式向前的
    // coordinate_frame = pcl::RangeImage::LASER_FRAME
    // X轴向前、Y轴向做、Z轴向上
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    // 指使用一个归一化的Z缓冲器来创建深度图像，但是如果想让临近点集都落在
    // 同一个像素单元，可以设置一个较高的值。
    // noiseLevel = 0.05可以理解为，深度距离值是通过查询点半径为5CM的源内包含的点以平均计算而得到的。
    float noiseLevel = 0.00;

    // 如果minRange > 0, 则所有模拟器所在的位置半径minRange内的邻近点都将被忽略，即为盲区
    float minRange = 0.0f;

    // 在裁剪图像时，如果borderSize > 0，将在图像周围留下当前视点不可见的边界
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << rangeImage << "\n";
}