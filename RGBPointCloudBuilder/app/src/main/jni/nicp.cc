#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/icp.hpp>

#include <nicp/imageutils.h> //
#include <nicp/pinholepointprojector.h> //
#include <nicp/depthimageconverterintegralimage.h>
#include <nicp/statscalculatorintegralimage.h>
#include <nicp/alignerprojective.h>

#include <vector>

#include "rgb-depth-sync/nicp.h"
#include "rgb-depth-sync/pcd_file_reader.h"

namespace rgb_depth_sync {
  NICP::NICP() {

    /*int imageScaling = 1;

    Eigen::Matrix3f cameraMatrix;
    cameraMatrix <<
        525.0f,   0.0f, 319.5f,
        0.0f, 525.0f, 239.5f,
        0.0f,   0.0f,   1.0f;

    // Create the PinholePointProjector
    nicp::PinholePointProjector pointProjector;
    pointProjector.setMinDistance(0.01f);
    pointProjector.setMaxDistance(4.5f);
    pointProjector.setCameraMatrix(cameraMatrix);
    pointProjector.setImageSize(480, 640);
    pointProjector.setTransform(Eigen::Isometry3f::Identity());
    pointProjector.scale(1.0f / imageScaling);

    // Create StatsCalculator and InformationMatrixCalculator
    nicp::StatsCalculatorIntegralImage statsCalculator;
    nicp::PointInformationMatrixCalculator pointInformationMatrixCalculator;
    nicp::NormalInformationMatrixCalculator normalInformationMatrixCalculator;

    statsCalculator.setMinImageRadius(20 / imageScaling);
    statsCalculator.setMaxImageRadius(40 / imageScaling);
    statsCalculator.setMinPoints(40 / imageScaling);
    statsCalculator.setCurvatureThreshold(0.2f);
    statsCalculator.setWorldRadius(0.1f);

    pointInformationMatrixCalculator.setCurvatureThreshold(0.02f);

    normalInformationMatrixCalculator.setCurvatureThreshold(0.02f);

    // Create DepthImageConverter
    nicp::DepthImageConverterIntegralImage converter(&pointProjector, &statsCalculator,
                                                     &pointInformationMatrixCalculator,
                                                     &normalInformationMatrixCalculator);

    // Create CorrespondenceFinder
    nicp::CorrespondenceFinderProjective correspondenceFinder;
    correspondenceFinder.setImageSize(pointProjector.imageRows(), pointProjector.imageCols());
    correspondenceFinder.setInlierDistanceThreshold(0.5f);
    correspondenceFinder.setInlierNormalAngularThreshold(0.95f);
    correspondenceFinder.setFlatCurvatureThreshold(0.02f);

    // Create Linearizer and Aligner
    nicp::Linearizer linearizer;
    nicp::AlignerProjective aligner;

    linearizer.setInlierMaxChi2(9e3);
    linearizer.setRobustKernel(true);
    linearizer.setZScaling(true);
    linearizer.setAligner(&aligner);

    aligner.setOuterIterations(10);
    aligner.setLambda(1e3);
    aligner.setProjector(&pointProjector);
    aligner.setCorrespondenceFinder(&correspondenceFinder);
    aligner.setLinearizer(&linearizer);

    // Get clouds from depth images
    Eigen::Isometry3f initialGuess = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f sensorOffset = Eigen::Isometry3f::Identity();
    nicp::RawDepthImage rawDepth;
    nicp::DepthImage depth, scaledDepth;
    nicp::Cloud referenceCloud, currentCloud, globalCloud;

    rawDepth = cv::imread("/storage/emulated/0/Documents/RGBPointCloudBuilder/NICP/reference_depth_image.png", -1);

    nicp::DepthImage_convert_16UC1_to_32FC1(depth, rawDepth, 0.001f);
    nicp::DepthImage_scale(scaledDepth, depth, imageScaling);
    //converter.compute(referenceCloud, scaledDepth, sensorOffset);

    rawDepth = cv::imread("/storage/emulated/0/Documents/RGBPointCloudBuilder/NICP/current_depth_image.png", -1);

    nicp::DepthImage_convert_16UC1_to_32FC1(depth, rawDepth, 0.001f);
    nicp::DepthImage_scale(scaledDepth, depth, imageScaling);
    converter.compute(currentCloud, scaledDepth, sensorOffset);

    LOGE("ICP size : %i", currentCloud.size());
    LOGE("ICP cloud size : %i", currentCloud._points.size());


    currentCloud._points.data();*/

    PCDFileReader* pcd_file_reader_ = new rgb_depth_sync::PCDFileReader();
    pcd_file_reader_->ReadFile("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD_dist/00001.pcd");

    pcd_file_reader_->GetPointsWithoutRGB();
    LOGE("ICP pcd size : %i", pcd_file_reader_->GetPointsWithoutRGB().size());

    cv::Mat openCVPointCloud(3, pcd_file_reader_->GetPointsWithoutRGB().size(), CV_64FC1);

    memcpy(openCVPointCloud.data, &(pcd_file_reader_->GetPointsWithoutRGB())[0], pcd_file_reader_->GetPointsWithoutRGB().size());

    /*currentCloud._normals.clear();
    currentCloud._normals.resize(pcd_file_reader_->GetPointsWithoutRGB().size());

    LOGE("ICP after size : %i", currentCloud.size());
    LOGE("ICP after cloud size : %i", currentCloud._points.size());

    //referenceCloud._normals = pcd_file_reader_->GetPointsWithoutRGB();


    // Perform the registration
    aligner.setReferenceCloud(&referenceCloud);
    aligner.setCurrentCloud(&currentCloud);
    aligner.setInitialGuess(initialGuess);
    aligner.setSensorOffset(sensorOffset);
    aligner.align();*/

    Eigen::Isometry3f icpPose = aligner.T();

    Eigen::Quaternionf eigen_rot(icpPose.rotation());
    glm::vec3 translation = glm::vec3(icpPose.translation().x(), icpPose.translation().y(),
                                      icpPose.translation().z());
    glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                   eigen_rot.z());
    LOGE("NICP pose:");
    LOGE("translation : %f %f %f", translation[0], translation[1], translation[2]);
    LOGE("rotation : %f %f %f %f", rotation[0], rotation[1], rotation[2], rotation[3]);
  }
}

