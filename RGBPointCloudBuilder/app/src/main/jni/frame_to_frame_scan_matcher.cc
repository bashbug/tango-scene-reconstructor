#include "rgb-depth-sync/frame_to_frame_scan_matcher.h"

namespace rgb_depth_sync {

  FrameToFrameScanMatcher::FrameToFrameScanMatcher() {
  }

  FrameToFrameScanMatcher::~FrameToFrameScanMatcher(){}

  void FrameToFrameScanMatcher::Init(PCDContainer* pcd_container) {
    loop_closures_count_ = 0;
    pcd_container_ = pcd_container;
    // allocating optimizer
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setVerbose(false);
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    // allocating solver
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(solver);
    //Set the initial Levenberg-Marquardt lambda
    solverLevenberg->setUserLambdaInit(1);
    optimizer_->setAlgorithm(solverLevenberg);

    information_.setIdentity();
    id_ = 0;
    first_pose_ = true;
    graph_counter_ = 0;
    is_running_ = false;

    InitGraph();
  }

  void FrameToFrameScanMatcher::InitGraph() {
    last_index_ = pcd_container_->GetPCDContainerLastIndex();
    Eigen::Isometry3d odometry_pose;
    for (int i = 0; i <= last_index_; i++) {
      Eigen::Isometry3d odometry_pose = util::CastGLMToEigenPosed(pcd_container_->pcd_container_[i]->GetPose());
      // add node to the pose graph
      id_ = AddNode(odometry_pose);
      if(id_ > 0) {
        // add edge to the pose graph
        AddEdge(id_-1, id_);
      }
    }
  }

  void FrameToFrameScanMatcher::Optimize() {
    DetectLoopClosures();
    OptimizeGraph();
  }

  void FrameToFrameScanMatcher::DetectLoopClosures() {
    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;

    for (int current = 0; current <= last_index_; current++) {

      curr_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[current]->GetPose());

      orderedNeighbors.clear();

      for (int previous = 0; previous < current - 5; previous++) {

        float translation_distance = 100 * GetDistance(
            pcd_container_->pcd_container_[current]->GetTranslation(),
            pcd_container_->pcd_container_[previous]->GetTranslation());

        if (translation_distance <= 10) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance;
          neighbor.id = previous;
          orderedNeighbors.push_back(neighbor);
        }
      }

      float overlap = 0;
      std::sort(orderedNeighbors.begin(), orderedNeighbors.end());

      for (int j = 0; j < orderedNeighbors.size(); j++) {

        std::clock_t start = std::clock();
        Eigen::Isometry3f loop_pose = Match(&overlap, pcd_container_->pcd_container_[orderedNeighbors[j].id]->GetPointCloud(),
                                                      pcd_container_->pcd_container_[current]->GetPointCloud(),
                                                      pcd_container_->pcd_container_[orderedNeighbors[j].id]->GetPose(),
                                                      pcd_container_->pcd_container_[current]->GetPose());

        if(isnan(overlap))
          continue;

        int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
        LOGE("Scan matcher  ---------- time %i", diff);
        LOGE("frames %i : %i dist : %f", current, orderedNeighbors[j].id, orderedNeighbors[j].distance);
        LOGE("overlap: %f", overlap);

        float distance_after = 100 * GetDistance(glm::vec3(0, 0, 0),
                                                 glm::vec3(loop_pose.translation().x(),
                                                           loop_pose.translation().y(),
                                                           loop_pose.translation().z()));

        if (overlap >= 0.80f && std::fabs(distance_after - orderedNeighbors[j].distance) <= 5.0f) {
          LOGE("GOOD LOOP");
          loop_closures_count_++;
          AddLoopClosure(orderedNeighbors[j].id, current, loop_pose, orderedNeighbors[j].distance);
        }
      }
    }
  }

  void FrameToFrameScanMatcher::OptimizeGraph() {
    LOGE("optimiziation start...");

    SaveGraph();

    optimizer_->initializeOptimization();
    // run optimization for 40 iterations
    optimizer_->optimize(40);
    std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> all_poses = GetPoses();
    LOGE("Pose graph size: %i", all_poses.size());
    LOGE("Loop clousures: %i", loop_closures_count_);

    for (int i = 0; i < all_poses.size(); i++) {
      glm::mat4 icppose_glm = util::ConvertEigenToGLMPose(all_poses[i]);
      glm::vec3 icp_translation = util::GetTranslationFromMatrix(icppose_glm);
      glm::quat icp_rotation = util::GetRotationFromMatrix(icppose_glm);
      pcd_container_->pcd_container_[i]->SetSMPose(all_poses[i]);
      pcd_container_->pcd_container_[i]->SetTranslationSM(icp_translation);
      pcd_container_->pcd_container_[i]->SetRotationSM(icp_rotation);
    }

    SaveGraph();

    LOGE("optimiziation stop...");
  }

  Eigen::Isometry3f FrameToFrameScanMatcher::Match(float* overlap,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_prev,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_curr,
                                        const glm::mat4& glm_odometryPose_prev,
                                        const glm::mat4& glm_odometryPose_curr) {

    Eigen::Isometry3f odometryPose_prev = util::ConvertGLMToEigenPose(glm_odometryPose_prev);
    Eigen::Isometry3f odometryPose_curr = util::ConvertGLMToEigenPose(glm_odometryPose_curr);

    Eigen::Vector3f translation_prev, translation_curr;

    float dist = sqrtf(static_cast<float>((frame_prev->sensor_origin_[0] - frame_curr->sensor_origin_[0])*(frame_prev->sensor_origin_[0] - frame_curr->sensor_origin_[0]) +
                                          (frame_prev->sensor_origin_[1] - frame_curr->sensor_origin_[1])*(frame_prev->sensor_origin_[1] - frame_curr->sensor_origin_[1]) +
                                          (frame_prev->sensor_origin_[2] - frame_curr->sensor_origin_[2])*(frame_prev->sensor_origin_[2] - frame_curr->sensor_origin_[2])));

    std::cout << "DIST BEFORE " << dist << std::endl;

    ProjectiveScanMatcher3d projective_scan_matcher;

    // set inital parameters of the scan matcher which is used for estimate the pose of the frames of loop closures
    projective_scan_matcher.parameters.searchRadius = 2;
    projective_scan_matcher.parameters.sourceImageStartStepSizeX = projective_scan_matcher.parameters.sourceImageStartStepSizeY = 1;
    projective_scan_matcher.parameters.maxDistanceStart = 0.5f;
    projective_scan_matcher.parameters.maxDistanceEnd = 0.02f;
    projective_scan_matcher.parameters.minScanOverlap = 1.0f;
    projective_scan_matcher.parameters.numIterations = 50;

    // set inital parameters of projective image
    SphericalProjectiveImage projective_image(util::Deg2Rad(0.5f), util::Deg2Rad(0.5f));
    projective_image.resizeImage(128, 77, 296, 143);
    projective_image.setCoordinateFrame(ProjectiveImage::CAMERA_FRAME);
    projective_image.clearPixels();
    projective_image.setSensorPose(Eigen::Isometry3f::Identity());

    // set previous frame
    projective_image.addPointsXYZ(frame_prev->points);
    projective_image.sortPixelPointsRegardingRange();

    Eigen::Isometry3f initialGuess;
    initialGuess = odometryPose_prev;

    std::clock_t start = std::clock();
    Eigen::Isometry3f icpPose = projective_scan_matcher.matchNewScan(overlap, projective_image, &initialGuess);
    int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    LOGE("SCAN MATCHER cpu time %i ms", diff);

    translation_prev = icpPose.translation();

    projective_image.clearPixels();
    projective_image.addPointsXYZ(frame_curr->points);
    projective_image.sortPixelPointsRegardingRange();

    initialGuess = icpPose * odometryPose_prev.inverse()*odometryPose_curr;

    start = std::clock();
    icpPose = projective_scan_matcher.matchNewScan(overlap, projective_image, &initialGuess);
    diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    LOGE("SCAN MATCHER cpu time %i ms", diff);

    translation_curr = icpPose.translation();

    dist = sqrtf(static_cast<float>((translation_prev[0] - translation_curr[0])*(translation_prev[0] - translation_curr[0]) +
                                    (translation_prev[1] - translation_curr[1])*(translation_prev[1] - translation_curr[1]) +
                                    (translation_prev[2] - translation_curr[2])*(translation_prev[2] - translation_curr[2])));

    std::cout << "DIST AFTER " << dist << std::endl;

    // get icp pose for given depth data
    return icpPose;
  }

  float FrameToFrameScanMatcher::GetDistance(const glm::vec3 &curr_depth_point, const glm::vec3 &trans_depth_point) {
    return sqrtf(static_cast<float>((curr_depth_point[0] - trans_depth_point[0])*(curr_depth_point[0] - trans_depth_point[0]) +
                                    (curr_depth_point[1] - trans_depth_point[1])*(curr_depth_point[1] - trans_depth_point[1]) +
                                    (curr_depth_point[2] - trans_depth_point[2])*(curr_depth_point[2] - trans_depth_point[2])));
  }

  g2o::VertexSE3* FrameToFrameScanMatcher::GetNode(int id) {
    return dynamic_cast<g2o::VertexSE3*>(optimizer_->vertices()[id]);
  }

  int FrameToFrameScanMatcher::AddNode(Eigen::Isometry3d pose) {
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

  void FrameToFrameScanMatcher::AddEdge(int prev_id, int cur_id) {

    g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(prev_id));
    g2o::VertexSE3* cur_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(cur_id ));

    g2o::EdgeSE3* e = new g2o::EdgeSE3;

    e->setVertex(0, prev_vertex);
    e->setVertex(1, cur_vertex);

    Eigen::Isometry3d relative_transformation = prev_vertex->estimate().inverse() * cur_vertex->estimate();

    e->setMeasurement(relative_transformation);
    e->setInformation(information_);
    optimizer_->addEdge(e);
  }

  void FrameToFrameScanMatcher::AddLoopClosure(const int &prev_id, const int &cur_id, const Eigen::Isometry3f &relative_position_f, int confidence) {

    Eigen::Isometry3d relative_position = util::CastIsometry3fTo3d(relative_position_f);

    g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(prev_id));
    g2o::VertexSE3* cur_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(cur_id ));

    g2o::EdgeSE3* e = new g2o::EdgeSE3;

    e->setVertex(0, prev_vertex);
    e->setVertex(1, cur_vertex);

    e->setMeasurement(relative_position);
    e->setInformation(confidence*information_);
    optimizer_->addEdge(e);
  }

  Eigen::Isometry3f FrameToFrameScanMatcher::GetPose(int id) {
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

  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > FrameToFrameScanMatcher::GetPoses() {
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

  void FrameToFrameScanMatcher::SaveGraph() {
    char filename[1024];
    sprintf(filename, "/storage/emulated/10/Documents/RGBPointCloudBuilder/Graph/graph_%05d.g2o", graph_counter_);
    optimizer_->save(filename, 0);
    graph_counter_++;
  }
}
