//
// Created by anastasia on 28.12.15.
//

#ifndef RGBPOINTCLOUDBUILDER_SLAM3D_H
#define RGBPOINTCLOUDBUILDER_SLAM3D_H

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/sparse_optimizer.h"

#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <Eigen/Geometry>

#include "rgb-depth-sync/util.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

namespace rgb_depth_sync {

  class Slam3D {
    public:
      Slam3D();
      ~Slam3D();
      int AddNode(Eigen::Isometry3d pose);
      void AddEdge(int prev_id, int cur_id);
      void AddLoopClosure(const int &prev_id, const int &cur_id, const Eigen::Isometry3f &relative_position_f, int confidence);
      g2o::VertexSE3* GetNode(int id);
      Eigen::Isometry3f GetPose(int id);
      void OptimizeGraph();
      std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> GetPoses();
      void SaveGraph();
    private:
      g2o::SparseOptimizer* optimizer_;
      g2o::BlockSolverX::LinearSolverType* linear_solver_;
      g2o::BlockSolverX* solver_ptr_;
      Eigen::Matrix<double, 6, 6> information_;
      int id_;
      int counter_;
      int accuracy_;
      bool first_pose_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_SLAM3D_H
