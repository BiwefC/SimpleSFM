#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "SLAMBase.hpp"
#include "Graph.hpp"

int main( int argc, char** argv )
{
    int startIndex = 1;
    int endIndex = 782;

    Graph graph;

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    Frame::Ptr lastFrame(new Frame(currIndex, "../../data")); // 上一帧数据
    // 我们总是在比较currFrame和lastFrame
    Camera_Intrinsic_Parameters camera;
    camera.scale = 1000;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;

    lastFrame->ComputeFeatAndDesp();
    PointCloud::Ptr cloud = Image2PointCloud(lastFrame->rgb, lastFrame->depth, camera);

    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = true;

    int min_inliers = 5;
    double max_norm = 0.3;

    graph.InsertFirstFrame(lastFrame);

    for(currIndex = startIndex+1; currIndex < endIndex; currIndex++)
    {
        cout << "Reading files " << currIndex << endl;
        Frame::Ptr currFrame(new Frame(currIndex, "../../data")); // 读取currFrame
        currFrame->ComputeFeatAndDesp();
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
        //cloud = UpdatePointCloud( cloud, currFrame, T, camera );
        graph.InsertKeyFrame(currFrame, result);
    }

    // pcl::io::savePCDFile( "data/result.pcd", *cloud );
    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<graph.optimizer.vertices().size()<<endl;
    graph.optimizer.save("../data/result_before.g2o");
    graph.optimizer.initializeOptimization();
    graph.optimizer.optimize( 100 ); //可以指定优化步数
    graph.optimizer.save( "../data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    PointCloud::Ptr output ( new PointCloud() );
    PointCloud::Ptr tmp ( new PointCloud() );

    pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

    double gridsize = 0.01; //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=0; i<graph.keyframes.size(); i++)
    {
        // 从g2o里取出一帧
        std::cout << "Frame " << i << " added!" << std::endl;
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(graph.optimizer.vertex(graph.keyframes[i]->frameID));
        if(vertex == NULL){
            continue;
        }
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        PointCloud::Ptr newCloud = Image2PointCloud(graph.keyframes[i]->rgb, graph.keyframes[i]->depth, camera); //转成点云
        // 以下是滤波
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // 把点云变换后加入全局地图中
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );
    //存储

    viewer.showCloud(output);
    pcl::io::savePCDFile( "../data/result.pcd", *tmp );

    cout<<"Final map is saved."<<endl;
    graph.optimizer.clear();
    while(1);


    return 0;
}
