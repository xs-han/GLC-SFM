//
// Created by xushen on 10/23/17.
//

#ifndef MV_SLAM_OPTIMIZER_H
#define MV_SLAM_OPTIMIZER_H

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "KeyFrame.h"
#include "MapPoint.h"

class Optimizer {
public:
    static void GlobalBundleAdjustment(vector<KeyFrame *> &kfs, vector<MapPoint *> & pc,Mat & cameraMatrix);
};


#endif //MV_SLAM_OPTIMIZER_H
