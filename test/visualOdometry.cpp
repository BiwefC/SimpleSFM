#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "SLAMBase.hpp"


int main( int argc, char** argv )
{
    int startIndex = 1;
    int endIndex = 782;

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    Frame lastFrame = Frame(currIndex, "../../data"); // 上一帧数据
    // 我们总是在比较currFrame和lastFrame
    Camera_Intrinsic_Parameters camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;

    lastFrame.ComputeFeatAndDesp();
    PointCloud::Ptr cloud = Image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = true;

    int min_inliers = 5;
    double max_norm = 0.3;

    for(currIndex = startIndex+1; currIndex < endIndex; currIndex++)
    {
        cout << "Reading files " << currIndex << endl;
        Frame currFrame = Frame(currIndex, "../../data"); // 读取currFrame
        currFrame.ComputeFeatAndDesp();
        // 比较currFrame 和 lastFrame
        Result_of_PnP result = MatchAndRansac(lastFrame, currFrame, camera);
        if (result.inlPoint < min_inliers) //inliers不够，放弃该帧
            continue;
        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;
        Eigen::Isometry3d T = RvecTvec2Mat(result.rvec, result.tvec);
        cout<<"T = "<<T.matrix()<<endl;

        //cloud = joinPointCloud( cloud, currFrame, T.inverse(), camera );
        cloud = UpdatePointCloud( cloud, currFrame, T, camera );

        if ( visualize && (currIndex % 10 == 0) )
            viewer.showCloud( cloud );

        lastFrame = currFrame;
    }

    // pcl::io::savePCDFile( "data/result.pcd", *cloud );
    return 0;
}
