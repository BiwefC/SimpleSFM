#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "SLAMBase.hpp"
#include "Graph.hpp"
#include "Loop.hpp"

enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

CHECK_RESULT CompareWithKeyFrame(Frame::Ptr& frame1, Frame::Ptr& frame2, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera, bool is_loops = false);
void checkBoWLoops( vector<Frame::Ptr>& frames, Frame::Ptr& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera);
void checkNearbyLoops( vector<Frame::Ptr>& frames, Frame::Ptr& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera);
Graph graph;
Loop loop_inst;

int main( int argc, char** argv )
{
    int startIndex = 1;
    int endIndex = 782;

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

    int min_inlPoint = 5;
    double max_norm = 0.3;

    graph.InsertFirstFrame(lastFrame);

    double keyframe_th = 0.1;
    bool check_loop_closure = true;

    for(currIndex = startIndex+1; currIndex < endIndex; currIndex++){
        cout<<"Reading files "<<currIndex<<endl;
        Frame::Ptr currFrame(new Frame(currIndex, "../../data")); // 读取currFrame
        currFrame->ComputeFeatAndDesp();
        CHECK_RESULT result = CompareWithKeyFrame(graph.keyframes.back(), currFrame, graph.optimizer, camera); //匹配该帧与keyframes里最后一帧
        switch (result){ // 根据匹配结果不同采取不同策略
            case NOT_MATCHED:
                //没匹配上，直接跳过
                cout<<"Not enough inliers."<<endl;
                break;

            case TOO_FAR_AWAY:
                // 太近了，也直接跳
                cout<<"Too far away, may be an error."<<endl;
                break;

            case TOO_CLOSE:
                // 太远了，可能出错了
                cout<<"Too close, not a keyframe"<<endl;
                break;

            case KEYFRAME:
                cout<<"This is a new keyframe"<<endl;
                // 不远不近，刚好
                /**
                 * This is important!!
                 * This is important!!
                 * This is important!!
                 * (very important so I've said three times!)
                 */
                // 检测回环
                loop_inst.add(currFrame);
                if (check_loop_closure)
                {
                    vector<Frame::Ptr> frames_tmp = loop_inst.getPossibleLoops(currFrame);
                    checkNearbyLoops(graph.keyframes, currFrame, graph.optimizer, camera);
                    checkBoWLoops(frames_tmp, currFrame, graph.optimizer, camera);
                }
                // keyframes.push_back(currFrame);
                break;

            default:
                break;
        }
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
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(graph.optimizer.vertex( graph.keyframes[i]->frameID ));
        if(vertex == NULL){
            continue;
        }
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        PointCloud::Ptr newCloud = Image2PointCloud( graph.keyframes[i]->rgb, graph.keyframes[i]->depth, camera ); //转成点云
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
    // globalOptimizer.clear();
    while(1);

    return 0;
}


CHECK_RESULT CompareWithKeyFrame(Frame::Ptr& frame1, Frame::Ptr& frame2, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera, bool is_loops)
{
    static int min_inlPoint = 5;
    static double max_norm = 0.3;
    static double keyframe_th = 0.1;
    static double max_norm_lp = 2.0;

    Frame::Ptr frame_tmp = frame1;

    Result_of_PnP result = MatchAndRansac(frame1, frame2, camera);

    if(result.inlPoint < min_inlPoint)
    {
        return NOT_MATCHED;
    }

    double norm = normofTransform(result.rvec, result.tvec);

    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_th )
        return TOO_CLOSE;   // too adjacent frame
    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点部分
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        graph.AddVertex(frame2);
    }
    graph.AddEdge(frame_tmp, frame2, result);

    return KEYFRAME;
}


void checkNearbyLoops( vector<Frame::Ptr>& frames, Frame::Ptr& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera)
{
    static int nearby_loops = 5;

    // 就是把currFrame和 frames里末尾几个测一遍
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            CompareWithKeyFrame( frames[i], currFrame, opti, camera, true);
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            CompareWithKeyFrame( frames[i], currFrame, opti, camera, true);
        }
    }
}

void checkBoWLoops( vector<Frame::Ptr>& frames, Frame::Ptr& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera)
{
    static int random_loops = 5;
    // 随机取一些帧进行检测

    // no enough keyframes, check everyone
    for (size_t i=0; i<frames.size(); i++)
    {
        CompareWithKeyFrame( frames[i], currFrame, opti, camera, true);
    }
}