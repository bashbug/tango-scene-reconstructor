#include "rgb-depth-sync/scan_matcher.h"

namespace rgb_depth_sync {

  ScanMatcher::ScanMatcher() {

    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get the intrinsics for the color camera.");
    }
  }

  ScanMatcher::~ScanMatcher(){}

  Eigen::Isometry3f ScanMatcher::Match(const ProjectiveImage::ImagePixels& frame_prev,
                                       const ProjectiveImage::ImagePixels& frame_curr,
                                       const glm::mat4& glm_odometryPose_prev,
                                       const glm::mat4& glm_odometryPose_curr) {

    Eigen::Isometry3f odometryPose_prev = util::ConvertGLMToEigenPose(glm_odometryPose_prev);
    Eigen::Isometry3f odometryPose_curr = util::ConvertGLMToEigenPose(glm_odometryPose_curr);

    ProjectiveScanMatcher3d projective_scan_matcher;
    // set inital parameters of the scan matcher which is used for estimate the pose of the frames of loop closures
    projective_scan_matcher.parameters.searchRadius = 1;
    projective_scan_matcher.parameters.sourceImageStartStepSizeX = projective_scan_matcher.parameters.sourceImageStartStepSizeY = 2;
    projective_scan_matcher.parameters.maxDistanceStart = 0.5f;
    projective_scan_matcher.parameters.maxDistanceEnd = 0.02f;
    projective_scan_matcher.parameters.minScanOverlap = 1.0f;
    projective_scan_matcher.parameters.numIterations = 100;
    projective_scan_matcher.parameters.maxDistanceBetweenScans = 0.5;

    float angluarResolutionX = util::Deg2Rad(0.5f);
    float angularResolutionY = angluarResolutionX;

    Eigen::Isometry3f sensorPose = Eigen::Isometry3f::Identity();

    SphericalProjectiveImage projective_image(angluarResolutionX, angularResolutionY, sensorPose, ProjectiveImage::CAMERA_FRAME);
    projective_image.resizeImage(320, 180, 0, 0);
    projective_image.setImagePixels(frame_prev);

    projective_image.setSensorPose(Eigen::Isometry3f::Identity());

    Eigen::Isometry3f icpPose = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f initalGuess = odometryPose_prev;
    // get icp pose for given depth data
    icpPose = projective_scan_matcher.matchNewScan(projective_image, &initalGuess);
    Eigen::Quaternionf eigen_rot(icpPose.rotation());
    glm::vec3 translation = glm::vec3(icpPose.translation().x(), icpPose.translation().y(), icpPose.translation().z());
    glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());

    projective_image.clearPixels();
    projective_image.setImagePixels(frame_curr);

    initalGuess = icpPose * odometryPose_prev.inverse() * odometryPose_curr;
    // get icp pose for given depth data
    icpPose = projective_scan_matcher.matchNewScan(projective_image, &initalGuess);
    eigen_rot = icpPose.rotation();
    translation = glm::vec3(icpPose.translation().x(), icpPose.translation().y(), icpPose.translation().z());
    rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
    LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
    LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);

    return icpPose;
  }
}
