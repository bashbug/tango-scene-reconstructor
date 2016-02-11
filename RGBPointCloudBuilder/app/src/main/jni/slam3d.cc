//
// Created by anastasia on 28.12.15.
//

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "g2o/core/optimization_algorithm_levenberg.h"

#include "rgb-depth-sync/slam3d.h"

namespace rgb_depth_sync {

  Slam3D::Slam3D(PCDContainer* pcd_container, std::shared_ptr<std::mutex> pcd_mtx,
                 std::shared_ptr<std::condition_variable> consume_pcd,
                 std::shared_ptr<std::atomic<bool>> optimize_poses_process_started) {

    pcd_container_ = pcd_container;
    pcd_mtx_ = pcd_mtx;
    consume_pcd_ = consume_pcd;
    optimize_poses_process_started_ = optimize_poses_process_started;

    // allocating the optimizer
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(true);
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(solver);
    optimizer_->setAlgorithm(solverLevenberg);

    loop_closure_detector_ = new rgb_depth_sync::LoopClosureDetector(pcd_container);

    //information_ = Eigen::Matrix<double, 4, 4>::Identity();
    id_ = 0;
    optimize_poses_ = false;
    first_pose_ = true;
    counter_ = 0;
    start_OnPCDAvailable_thread_ = false;
  }

  Slam3D::~Slam3D() {
    delete optimizer_;
  }

  void Slam3D::StartOnFrameAvailableThread() {
    start_OnPCDAvailable_thread_ = true;
  }

  void Slam3D::StopOnFrameAvailableThread() {
    start_OnPCDAvailable_thread_ = false;
  }

  int Slam3D::OnPCDAvailable() {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);

    while(start_OnPCDAvailable_thread_) {
      consume_pcd_->wait(lock);
      if (!optimize_poses_) {
        int lastIndex = pcd_container_->GetPCDContainerLastIndex();

        odometryPose_ = util::ConvertGLMToEigenPose((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());
        odometryPose_d_ = util::CastIsometry3fTo3d(odometryPose_);

        // add node to the pose graph
        id_ = AddNode(odometryPose_d_);

        if(id_ > 0) {
          // add edge to the pose graph
          AddEdge(id_-1, id_);
        }

        // search for loop closures
        loop_closure_detector_->Compute(lastIndex);
      }
    }

    return -1;
  }

  g2o::VertexSE3* Slam3D::GetNode(int id) {
    return dynamic_cast<g2o::VertexSE3*>(optimizer_->vertices()[id]);
  }

  int Slam3D::AddNode(Eigen::Isometry3d pose) {
    id_ = optimizer_->vertices().size();

    g2o::VertexSE3* v = new g2o::VertexSE3;
    v->setId(id_);

    if (first_pose_) {
      v->setEstimate(pose);
      v->setFixed(true);
      first_pose_ = false;
    } else {
      v->setEstimate(pose);
    }

    optimizer_->addVertex(v);

    return id_;
  }

  void Slam3D::AddEdge(int prev_id, int cur_id) {

    g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(prev_id));
    g2o::VertexSE3* cur_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(cur_id ));

    g2o::EdgeSE3* e = new g2o::EdgeSE3;

    e->setVertex(0, prev_vertex);
    e->setVertex(1, cur_vertex);

    Eigen::Matrix4d relative_transformation = (prev_vertex->estimate().inverse() * cur_vertex->estimate()).matrix();
    g2o::SE3Quat measurement_mean(Eigen::Quaterniond(relative_transformation.block<3,3>(0,0)), relative_transformation.block<3,1>(0,3));

    e->setMeasurement(measurement_mean);
    e->setInformation(information_);
    optimizer_->addEdge(e);
  }

  void Slam3D::AddLoopClosure(const int &prev_id, const int &cur_id, const Eigen::Isometry3f &relative_position_f, int confidence) {

    Eigen::Isometry3d relative_position = util::CastIsometry3fTo3d(relative_position_f);

    g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(prev_id));
    g2o::VertexSE3* cur_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(cur_id ));

    g2o::EdgeSE3* e = new g2o::EdgeSE3;

    e->setVertex(0, prev_vertex);
    e->setVertex(1, cur_vertex);

    Eigen::Matrix4d relative_transformation = relative_position.matrix();
    Eigen::Quaterniond rotation(relative_position.rotation());
    Eigen::Vector3d translation(relative_position.translation());
    g2o::SE3Quat measurement_mean(rotation, translation);

    e->setMeasurement(measurement_mean);
    e->setInformation(confidence*information_);
    optimizer_->addEdge(e);
  }

  void Slam3D::OptimizeGraph() {
    LOGE("optimiziation start...");
    optimize_poses_ = true;

    // Add all loop closure poses to the graph
    loop_closure_detector_->GetLoopClosurePoses(&loop_closure_poses_);

    LOGE("Found %i loop_closures", loop_closure_poses_->size());

    for (it_ = loop_closure_poses_->begin(); it_ != loop_closure_poses_->end(); it_++) {
      // wait with .get() for async scan matcher computation
      AddLoopClosure(it_->first.first, it_->first.second, it_->second.second, it_->second.first);
    }

    optimizer_->initializeOptimization();

    // run optimization for 40 iterations
    optimizer_->optimize(40);

    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> all_poses = GetPoses();

    for (int i = 0; i < all_poses.size(); i++) {
      glm::mat4 icppose_glm = util::ConvertEigenToGLMPose(all_poses[i]);
      glm::vec3 icp_translation = util::GetTranslationFromMatrix(icppose_glm);
      glm::quat icp_rotation = util::GetRotationFromMatrix(icppose_glm);

      //icp_positions[i] = icp_translation;
      (*(pcd_container_->GetPCDContainer()))[i]->SetTranslation(icp_translation);
      (*(pcd_container_->GetPCDContainer()))[i]->SetRotation(icp_rotation);
    }

    optimize_poses_ = false;
    optimize_poses_process_started_ = std::make_shared<std::atomic<bool>>(false);
    LOGE("optimiziation stop...");
  }

  Eigen::Isometry3f Slam3D::GetPose(int id) {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(id));
    double optimizedPoseQuaternion[7];
    v->getEstimateData(optimizedPoseQuaternion);

    Eigen::Matrix4f optimizedPose;
    static double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

    qx=optimizedPoseQuaternion[3];
    qy=optimizedPoseQuaternion[4];
    qz=optimizedPoseQuaternion[5];
    qr=optimizedPoseQuaternion[6];
    qx2=qx*qx;
    qy2=qy*qy;
    qz2=qz*qz;
    qr2=qr*qr;

    optimizedPose(0,0)=qr2+qx2-qy2-qz2;
    optimizedPose(0,1)=2*(qx*qy-qr*qz);
    optimizedPose(0,2)=2*(qz*qx+qr*qy);
    optimizedPose(0,3)=optimizedPoseQuaternion[0];
    optimizedPose(1,0)=2*(qx*qy+qr*qz);
    optimizedPose(1,1)=qr2-qx2+qy2-qz2;
    optimizedPose(1,2)=2*(qy*qz-qr*qx);
    optimizedPose(1,3)=optimizedPoseQuaternion[1];
    optimizedPose(2,0)=2*(qz*qx-qr*qy);
    optimizedPose(2,1)=2*(qy*qz+qr*qx);
    optimizedPose(2,2)=qr2-qx2-qy2+qz2;
    optimizedPose(2,3)=optimizedPoseQuaternion[2];
    optimizedPose(3,0)=0;
    optimizedPose(3,1)=0;
    optimizedPose(3,2)=0;
    optimizedPose(3,3)=1;

    Eigen::Isometry3f tmp;
    tmp.matrix() = optimizedPose;

    return tmp;
  }

  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > Slam3D::GetPoses() {
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > poses;

    for(int i = 0; i < optimizer_->vertices().size(); i++) {
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(i));
      double optimizedPoseQuaternion[7];
      v->getEstimateData(optimizedPoseQuaternion);

      Eigen::Matrix4f optimizedPose;
      static double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

      qx=optimizedPoseQuaternion[3];
      qy=optimizedPoseQuaternion[4];
      qz=optimizedPoseQuaternion[5];
      qr=optimizedPoseQuaternion[6];
      qx2=qx*qx;
      qy2=qy*qy;
      qz2=qz*qz;
      qr2=qr*qr;

      optimizedPose(0,0)=qr2+qx2-qy2-qz2;
      optimizedPose(0,1)=2*(qx*qy-qr*qz);
      optimizedPose(0,2)=2*(qz*qx+qr*qy);
      optimizedPose(0,3)=optimizedPoseQuaternion[0];
      optimizedPose(1,0)=2*(qx*qy+qr*qz);
      optimizedPose(1,1)=qr2-qx2+qy2-qz2;
      optimizedPose(1,2)=2*(qy*qz-qr*qx);
      optimizedPose(1,3)=optimizedPoseQuaternion[1];
      optimizedPose(2,0)=2*(qz*qx-qr*qy);
      optimizedPose(2,1)=2*(qy*qz+qr*qx);
      optimizedPose(2,2)=qr2-qx2-qy2+qz2;
      optimizedPose(2,3)=optimizedPoseQuaternion[2];
      optimizedPose(3,0)=0;
      optimizedPose(3,1)=0;
      optimizedPose(3,2)=0;
      optimizedPose(3,3)=1;

      Eigen::Isometry3f tmp;
      tmp.matrix() = optimizedPose;

      poses.push_back(tmp);
    }
    return poses;
  }

  void Slam3D::SaveGraph() {
    char filename[1024];
    LOGE("write file %i start...", counter_);
    sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Graph/graph_%05d.g2o", counter_);
    optimizer_->save(filename, 0);
    counter_++;
  }
}
