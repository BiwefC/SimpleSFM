#include <iostream>

#include "SLAMBase.hpp"

int main(int argc, char** argv)
{
    // while(1);
    cv::Mat rgb, depth;

    rgb = cv::imread("../data/rgb1.png");
    depth = cv::imread("../data/depth1.png", -1);

    pcl::visualization::CloudViewer viewer("viewer");

    Camera_Intrinsic_Parameters camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;

    PointCloud::Ptr cloud = Image2PointCloud(rgb, depth, camera);
    pcl::io::savePCDFile( "../data/pointcloud.pcd", *cloud );
    viewer.showCloud( cloud );

    while(1);
    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}
