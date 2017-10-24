//
// Created by xushen on 10/23/17.
//

#include "Optimizer.h"

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF) {
    // Setup optimizer
    g2o::SparseOptimizer    optimizer;
    // 使用Cholmod中的线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    // 6*3 的参数
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
    // L-M 下降
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );

    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );
}
