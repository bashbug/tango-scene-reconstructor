#include "rgb-depth-sync/scan_matcher.h"

namespace rgb_depth_sync {

  ScanMatcher::ScanMatcher() {

    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get the intrinsics for the color camera.");
    }

    ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_DEPTH, &depth_camera_intrinsics_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get the intrinsics for the color camera.");
    }
  }

  ScanMatcher::~ScanMatcher(){}

  Eigen::Isometry3f ScanMatcher::Match2(float* overlap,
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

  Eigen::Isometry3f ScanMatcher::Match(float* overlap,
                                       const std::vector<float>& frame_prev,
                                       const std::vector<float>& frame_curr,
                                       const glm::mat4& glm_odometryPose_prev,
                                       const glm::mat4& glm_odometryPose_curr) {

    Eigen::Isometry3f odometryPose_prev = util::ConvertGLMToEigenPose(glm_odometryPose_prev);
    Eigen::Isometry3f odometryPose_curr = util::ConvertGLMToEigenPose(glm_odometryPose_curr);

    float angular_resolution_X = util::Deg2Rad(1.23f);
    float angular_resolution_Y = util::Deg2Rad(0.69f);

    Eigen::Isometry3f sensorPose = Eigen::Isometry3f::Identity();
    SphericalProjectiveImage projective_image(angular_resolution_X, angular_resolution_Y, sensorPose, ProjectiveImage::CAMERA_FRAME);

    projective_image.resizeImage(260, 260, 26, 26);

    ProjectiveScanMatcher3d projective_scan_matcher;

    // set inital parameters of the scan matcher which is used for estimate the pose of the frames of loop closures
    projective_scan_matcher.parameters.searchRadius = 1;
    projective_scan_matcher.parameters.sourceImageStartStepSizeX = projective_scan_matcher.parameters.sourceImageStartStepSizeY = 2;
    projective_scan_matcher.parameters.maxDistanceStart = 0.2f;
    projective_scan_matcher.parameters.maxDistanceEnd = 0.02f;
    projective_scan_matcher.parameters.numIterations = 100;
    projective_scan_matcher.parameters.maxDistanceBetweenScans = 0.5;

    projective_image.setSensorPose(Eigen::Isometry3f::Identity());

    int count = 0;
    for (size_t i = 0; i < frame_prev.size(); i += 3) {
      if (std::abs(frame_prev[i]) < 1e-4 &&
          std::abs(frame_prev[i + 1]) < 1e-4 &&
          std::abs(frame_prev[i + 2]) < 1e-4) {
        continue;
      }
      // adding new depth data to the projective_image_
      projective_image.addPoint(frame_prev[i], frame_prev[i+1], frame_prev[i+2], count, false);
      count++;
    }

    projective_image.sortPixelPointsRegardingRange();
    count = 0;

    Eigen::Isometry3f icpPose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f initalGuess = odometryPose_prev;
    // get icp pose for given depth data
    icpPose = projective_scan_matcher.matchNewScan(overlap, projective_image, &initalGuess);
    projective_image.clearPixels();

    for (size_t i = 0; i < frame_curr.size(); i += 3) {
      if (std::abs(frame_curr[i]) < 1e-4 &&
          std::abs(frame_curr[i + 1]) < 1e-4 &&
          std::abs(frame_curr[i + 2]) < 1e-4) {
        continue;
      }
      // adding new depth data to the projective_image_
      projective_image.addPoint(frame_curr[i], frame_curr[i+1], frame_curr[i+2], count, false);
      count++;
    }

    projective_image.sortPixelPointsRegardingRange();
    initalGuess = icpPose * odometryPose_prev.inverse() * odometryPose_curr;
    // get icp pose for given depth data
    icpPose = projective_scan_matcher.matchNewScan(overlap, projective_image, &initalGuess);
    return icpPose;
  }
}
