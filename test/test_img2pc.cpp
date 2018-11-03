#include <iostream>

#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
    // while(1);
    cv::Mat rgb, depth;

    rgb = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png", -1);

    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;

    PointCloud::Ptr cloud = image2PointCloud(rgb, depth, camera);
    pcl::io::savePCDFile( "../data/pointcloud.pcd", *cloud );
    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}