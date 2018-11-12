#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

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

#include "SLAMBase.hpp"

typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

class Graph
{
    public:
        vector<Frame::Ptr> keyframes;
        vector<Frame::Ptr> newframes;
        Frame::Ptr refframe;
        g2o::SparseOptimizer optimizer;

        vector<int> vertexIdx;

        Graph(void);
        void InsertFirstFrame(Frame::Ptr &frame);
        void InsertKeyFrame(Frame::Ptr &frame, Result_of_PnP &result);
        void AddVertex(Frame::Ptr &frame);
        void AddEdge(Frame::Ptr &frame1, Frame::Ptr &frame2, Result_of_PnP &result);
};

#endif