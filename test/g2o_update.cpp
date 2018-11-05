#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "SLAMBase.hpp"

 //g2o的头文件
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};

CHECK_RESULT CompareWithKeyFrame(Frame& frame1, Frame& frame2, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera, bool is_loops = false);
void checkRandomLoops( vector<Frame>& frames, Frame& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera);
void checkNearbyLoops( vector<Frame>& frames, Frame& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera);

int main( int argc, char** argv )
{
    int startIndex = 1;
    int endIndex = 782;

    std::vector<Frame> keyframes;

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

    int min_inlPoint = 5;
    double max_norm = 0.3;

    // 新增:有关g2o的初始化
    // 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(std::unique_ptr<SlamBlockSolver::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<SlamBlockSolver>(blockSolver));

    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm(solver);
    // 不要输出调试信息
    globalOptimizer.setVerbose(false);

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(currIndex);
    v->setEstimate(Eigen::Isometry3d::Identity()); //估计为单位矩阵
    v->setFixed(true); //第一个顶点固定，不用优化
    globalOptimizer.addVertex(v);

    keyframes.push_back(lastFrame);

    double keyframe_th = 0.1;
    bool check_loop_closure = true;

    for(currIndex = startIndex+1; currIndex < endIndex; currIndex++){
        cout<<"Reading files "<<currIndex<<endl;
        Frame currFrame = Frame(currIndex, "../../data"); // 读取currFrame
        currFrame.ComputeFeatAndDesp();
        CHECK_RESULT result = CompareWithKeyFrame( keyframes.back(), currFrame, globalOptimizer, camera); //匹配该帧与keyframes里最后一帧
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
                if (check_loop_closure)
                {
                    checkNearbyLoops(keyframes, currFrame, globalOptimizer, camera);
                    checkRandomLoops(keyframes, currFrame, globalOptimizer, camera);
                }
                keyframes.push_back(currFrame);
                break;

            default:
                break;
        }
    }

    // pcl::io::savePCDFile( "data/result.pcd", *cloud );
    // 优化所有边
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("../data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "../data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    PointCloud::Ptr output ( new PointCloud() );
    PointCloud::Ptr tmp ( new PointCloud() );

    pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

    double gridsize = 0.01; //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=1; i<keyframes.size(); i++)
    {
        // 从g2o里取出一帧
        std::cout << "Frame " << i << " added!" << std::endl;
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        if(vertex == NULL){
            continue;
        }
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        PointCloud::Ptr newCloud = Image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云
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


CHECK_RESULT CompareWithKeyFrame(Frame& frame1, Frame& frame2, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera, bool is_loops)
{
    static int min_inlPoint = 5;
    static double max_norm = 0.3;
    static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
    static double keyframe_th = 0.1;
    static double max_norm_lp = 2.0;

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
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( frame2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->vertices() [0] = opti.vertex( frame1.frameID );
    edge->vertices() [1] = opti.vertex( frame2.frameID );
    edge->setRobustKernel( robustKernel );
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information );
    // 边的估计即是pnp求解之结果
    Eigen::Isometry3d T = RvecTvec2Mat( result.rvec, result.tvec );
    edge->setMeasurement( T.inverse() );
    // 将此边加入图中
    opti.addEdge(edge);
    return KEYFRAME;
}


void checkNearbyLoops( vector<Frame>& frames, Frame& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera)
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

void checkRandomLoops( vector<Frame>& frames, Frame& currFrame, g2o::SparseOptimizer& opti, Camera_Intrinsic_Parameters camera)
{
    static int random_loops = 5;
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测

    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            CompareWithKeyFrame( frames[i], currFrame, opti, camera, true);
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            CompareWithKeyFrame( frames[index], currFrame, opti, camera, true);
        }
    }
}