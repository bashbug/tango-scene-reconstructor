//
// Created by anastasia on 28.12.15.
//

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "g2o/core/optimization_algorithm_levenberg.h"

#include "rgb-depth-sync/slam3d.h"

namespace rgb_depth_sync {

  Slam3D::Slam3D() {

    // allocating the optimizer
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(true);
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(solver);
    optimizer_->setAlgorithm(solverLevenberg);

    information_ = Eigen::Matrix<double, 6, 6>::Identity();
    id_ = 0;
    first_pose_ = true;
    counter_ = 1;
    accuracy_ = 305;
  }

  Slam3D::~Slam3D() {
    delete optimizer_;
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
    e->setInformation(confidence*10*information_);
    optimizer_->addEdge(e);
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

  void Slam3D::OptimizeGraph() {

    optimizer_->initializeOptimization();

    //initial Levenberg-Marquardt lambda
    //optimizer_->setUserLambdaInit(0.01);

    // run optimization for 40 iterations
    optimizer_->optimize(400);
  }

  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> Slam3D::GetPoses() {
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> poses;

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
