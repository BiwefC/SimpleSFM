#include "Graph.hpp"

Graph::Graph(void)
{
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(std::unique_ptr<SlamBlockSolver::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<SlamBlockSolver>(blockSolver));

    optimizer.setAlgorithm(solver);
    // 不要输出调试信息
    optimizer.setVerbose(false);
}

void Graph::InsertFirstFrame(Frame::Ptr &frame)
{
    keyframes.push_back(frame);
    refframe = frame;
    vertexIdx.push_back(frame->frameID);

    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(frame->frameID);
    v->setEstimate(Eigen::Isometry3d::Identity());
    v->setFixed(true);
    optimizer.addVertex(v);
    return;
}

void Graph::InsertKeyFrame(Frame::Ptr &frame, Result_of_PnP &result)
{
    // cout<<"adding keyframe "<<frame->frameID<<" with ref to "<<refframe->frameID<<endl;
    // newframes.push_back(frame);
    // keyframes.push_back(frame);
    // vertexIdx.push_back(frame->frameID);

    // add the vertex
    AddVertex(frame);

    // and the edge with refframe
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 注意边的赋值有些绕，详见EdgeSE3的误差计算方式
    edge->vertices() [0] = optimizer.vertex(refframe->frameID);
    edge->vertices() [1] = optimizer.vertex(frame->frameID);

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information);
    // 边的估计即是pnp求解之结果
    edge->setMeasurement(RvecTvec2Mat(result.rvec, result.tvec ).inverse());
    // 将此边加入图中
    optimizer.addEdge(edge);

    // set ref frame to current
    refframe = frame;

    return;
}

void Graph::AddVertex(Frame::Ptr &frame)
{
    cout<<"adding keyframe "<<frame->frameID<<" with ref to "<<refframe->frameID<<endl;
    newframes.push_back(frame);
    keyframes.push_back(frame);
    vertexIdx.push_back(frame->frameID);

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(frame->frameID);
    v->setEstimate( Eigen::Isometry3d::Identity());
    v->setFixed(false);
    optimizer.addVertex(v);
}

void Graph::AddEdge(Frame::Ptr &frame1, Frame::Ptr &frame2, Result_of_PnP &result)
{
    static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );
    // and the edge with refframe
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 注意边的赋值有些绕，详见EdgeSE3的误差计算方式
    edge->vertices() [0] = optimizer.vertex(frame1->frameID);
    edge->vertices() [1] = optimizer.vertex(frame2->frameID);
    edge->setRobustKernel( robustKernel );

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation( information);
    // 边的估计即是pnp求解之结果
    edge->setMeasurement(RvecTvec2Mat(result.rvec, result.tvec ).inverse());
    // 将此边加入图中
    optimizer.addEdge(edge);
}