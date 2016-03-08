# RGBPointCloudBuilder

This project is based on the samples from the Google Tango API. You can find it at https://github.com/googlesamples/tango-examples-c

### Requirements
- android ndk r10e
- pcl 1.6
- opencv 3.1.0
- g2o

### Features
**Outlier filtering**
- pcl::StatisticalOutlierRemoval is used to remove outliers

**Sync image with point cloud**
- Using pose of image and point cloud timestamp.

**RGBD visualization**
- Merged rgb point clouds in real time and visualize with OpenGL.
- Using voxel hashing.

**Loop Closure Detection**
- Heuristik: Frames with a certain distance will matched. 
- The matching process (point-to-plane ICP) returns a relative transformation.

**Optimize Poses**
- Create a pose graph with tango and loop closure poses.
- Optimize graph with g2o (CSparse linear solver with Levenberg-Marquardt method)

**Store Point Clouds**
- Store rgb point cloud as *.PCD binary file in ../Documents/RGBPointCloudBuilder/PCD/

