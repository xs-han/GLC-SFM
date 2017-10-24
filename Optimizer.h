//
// Created by xushen on 10/23/17.
//

#ifndef MV_SLAM_OPTIMIZER_H
#define MV_SLAM_OPTIMIZER_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "KeyFrame.h"

class Optimizer {
    void static LocalBundleAdjustment(KeyFrame * pKF);
};


#endif //MV_SLAM_OPTIMIZER_H
