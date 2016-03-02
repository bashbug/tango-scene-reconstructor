//
// Created by anastasia on 28.12.15.
//

#ifndef RGBPOINTCLOUDBUILDER_SLAM3D_H
#define RGBPOINTCLOUDBUILDER_SLAM3D_H

#include <map>
#include <utility>
#include <future>
#include <mutex>
#include <condition_variable>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "g2o/core/optimization_algorithm_factory.h"
#include <tango-gl/util.h>
#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/loop_closure_detector.h"

namespace rgb_depth_sync {

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  typedef std::pair<int, int> key;
  typedef std::pair<int, Eigen::Isometry3f > value;

  class Slam3D {
    public:
      Slam3D(PCDContainer* pcd_container, std::shared_ptr<std::mutex> pcd_mtx,
             std::shared_ptr<std::condition_variable> consume_pcd,
             std::shared_ptr<std::atomic<bool>> optimize_poses_process_started);
      ~Slam3D();
      int AddNode(Eigen::Isometry3d pose);
      void AddEdge(int prev_id, int cur_id);
      void AddLoopClosure(const int &prev_id, const int &cur_id, const Eigen::Isometry3f &relative_position_f, int confidence);
      g2o::VertexSE3* GetNode(int id);
      Eigen::Isometry3f GetPose(int id);
      void OptimizeGraph();
      void StartOnFrameAvailableThread();
      void StopOnFrameAvailableThread();
      std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > GetPoses();
      void SaveGraph();
      int OnPCDAvailable();
    private:
      LoopClosureDetector* loop_closure_detector_;
      g2o::SparseOptimizer* optimizer_;
      g2o::BlockSolverX::LinearSolverType* linear_solver_;
      g2o::BlockSolverX* solver_ptr_;
      Eigen::Matrix<double, 6, 6> information_;
      std::map<key, value>* loop_closure_poses_;
      std::map<key, value>::iterator it_;
      int id_;
      int counter_;
      Eigen::Isometry3f odometryPose_;
      Eigen::Isometry3d odometryPose_d_;
      bool optimize_poses_;
      bool first_pose_;
      bool start_OnPCDAvailable_thread_;
      PCDContainer* pcd_container_;
      std::shared_ptr<std::mutex> pcd_mtx_;
      std::shared_ptr<std::condition_variable> consume_pcd_;
      std::shared_ptr<std::atomic<bool>> optimize_poses_process_started_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_SLAM3D_H
