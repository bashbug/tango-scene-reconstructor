# RGBPointCloudBuilder

This project is based on the samples from the Google Tango API. You can find it at https://github.com/googlesamples/tango-examples-c

It uses to different approaches a frame-to-frame (FTFSM) and a mutli-frame scan matching (MFSM) to optimize the RGB point cloud object model. Both methods use the Tango VIO poses as inital guess and g2o as a graph-based optimization system. An example model of an RGB point cloud alignment with Tango VIO poses

<img alt="TANGO VIO" src="https://github.com/bashbug/TangoProject/blob/master/img/tango_mesh_and_pose_graph_with_red_loops.png">

and its given pose graph generated with g2o. The red lines are detected loop closures by the FTFMS method. The result of the FTFMS optimized graph and model shows a more accurate alginment.

<img alt="FTFSM" src="https://github.com/bashbug/TangoProject/blob/master/img/ftfsm_mesh_and_pose_graph_with_red_loops.png">

Also the MFSM aligns the RGB point clouds more accurately.
<img alt="MFSM" src="https://github.com/bashbug/TangoProject/blob/master/img/mfsm_mesh.png" width="280" align="right">

</br>
</br>
</br>
</br>

### Requirements
- Android NDK r10e http://developer.android.com/ndk/downloads/index.html
- PCL 1.8 https://github.com/PointCloudLibrary/pcl
- OpenCV for Android 3.1.0 http://opencv.org/downloads.html
- g2o https://github.com/RainerKuemmerle/g2o
- Boost 1.55.0 http://www.boost.org
- Eigen 3.2.8 http://eigen.tuxfamily.org
- FLANN 1.8.4 http://www.cs.ubc.ca/research/flann/#download
- projective-scan-matcher-3d (not open source)
- multiframe-scan-matcher-3d (not open source)

### Features
**Outlier filtering**
- pcl::StatisticalOutlierRemoval is used to remove outliers

**Sync image with point cloud**
- Using pose of image and point cloud timestamp.

**RGBD visualization**
- Merged RGB point clouds in real time and visualize with OpenGL.
- Using voxel grid for downsampling and removing of douplicates

**Frame-to-Frame Scan Matching (FTFSM)**

***Loop Closure Detection***
- Heuristik: Frames with a certain distance will be matched. 

***Pose estmation***
- The matching process (point-to-plane ICP) returns a relative transformation (used ICP is not open source).

***Pose optimization***
- Create a pose graph with tango and loop closure poses.
- Optimize graph with g2o (CSparse linear solver with Levenberg-Marquardt method) by minimizing the distance between the loop closure and Tango VIO poses.

**Multi-Frame Scan Matching (MFSM)**

***Surface correspondence estmation***
- Normal estimation with pcl::NormalEstimationOMP.
- For a given point of a point cloud all corresponding points are estimated by searching each kdTree its nearest neighbor.

***Pose optimization***
- Create a pose graph with Tango VIO and loop closure poses. Each pose has surface correspondences as spatial constraints.
- Optimize graph with g2o (CSparse linear solver with Levenberg-Marquardt method) by minimizing the distances between the spatial constraints.

**Store Point Clouds**
- Store RGB point cloud as *.PCD binary file in 
  _/Documents/RGBPointCloudBuilder/yyyyMMddHHmmss/PCD
- Store EGB point cloud object models as FTFSM.PCD and MFSM.PCD binary file in 
  _/Documents/RGBPointCloudBuilder/yyyyMMddHHmmss/Mesh/

**Zip&Share current scan**
- Current stored RGB point clouds and object models are zipped and sharable to google drive or via other installed apps.


