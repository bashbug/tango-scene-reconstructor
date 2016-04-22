/**
 * Frame-to-Frame Scan Matcher:
 *
 * Post-processing method to optimize poses with range data.
 * Graph-based 3D SLAM which uses a heuristic to detect loop closures, calculates the relative
 * transformation between them with a point-to-plane ICP and optimizes the pose with g2o.
 * see low2004linear, park2003accurate, holz2015registration
 *
 * g2o is used as a graph optimization system with Levenberg-Marquardt and a CSparse solver
 * see grisetti2010tutorial, grisetti2011g2o, kummerle2011g
 */

#ifndef RGBPOINTCLOUDBUILDER_SCANMATCHER_H
#define RGBPOINTCLOUDBUILDER_SCANMATCHER_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <projectiveScanMatcher3d/projectiveScanMatcher3d.h>
#include <projectiveImage/sphericalProjectiveImage.h>
#include <tango-gl/util.h>
#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
  typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  struct NeighborWithDistance {
    float distance;
    int id;
    bool fm = false;
    int no_matches = 0;
    bool operator < (const NeighborWithDistance& other) const { return distance < other.distance; }
  };

  class FrameToFrameScanMatcher {
    public:
      FrameToFrameScanMatcher();
      ~FrameToFrameScanMatcher();
      void Init(PCDContainer* pcd_container);
      void Optimize();
    private:
      Eigen::Isometry3f Match(float* overlap,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_prev,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_curr,
                            const glm::mat4& glm_odometryPose_prev,
                            const glm::mat4& glm_odometryPose_curr);
      void InitGraph();
      void DetectLoopClosures();
      void OptimizeGraph();
      void AddEdge(int prev_id, int cur_id);
      int AddNode(Eigen::Isometry3d pose);
      void AddLoopClosure(const int &prev_id, const int &cur_id, const Eigen::Isometry3f &relative_position_f, int confidence);
      float GetDistance(const glm::vec3 &curr_trans, const glm::vec3 &prev_trans);
      g2o::VertexSE3* GetNode(int id);
      Eigen::Isometry3f GetPose(int id);
      std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > GetPoses();
      void SaveGraph();
      g2o::SparseOptimizer* optimizer_;
      g2o::BlockSolverX::LinearSolverType* linear_solver_;
      g2o::BlockSolverX* solver_ptr_;
      Eigen::Matrix<double, 6, 6> information_;
      int last_index_, id_, graph_counter_, loop_closures_count_;
      float distance_;
      glm::quat curr_rotation_;
      glm::quat prev_rotation_;
      Eigen::Isometry3d odometry_pose_;
      PCDContainer* pcd_container_;
      bool is_running_, first_pose_;
  };
}



#endif //RGBPOINTCLOUDBUILDER_SCANMATCHER_H
