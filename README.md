# RGBPointCloudBuilder

This project is based on the samples from the Google Tango API. You can find it at https://github.com/googlesamples/tango-examples-c

### Features
**Sync image with point cloud**
- Using pose of image and point cloud timestamp.
- Store rgb point cloud as timestamp.PCD binary file in ../Documents/RGBPointCloudBuilder/PCD/

**Send Point Clouds**
- Socket to send binary rgb point cloud data to a server (uses SocketCommunication/socket.cpp)

**Stores PPM image**
- Converts YUV image buffer to RGB
- Saves RGB image as a PPM file in ../Documents/RGBPointCloudBuilder/PPM/

### Todo
**visualize with OpenGL**
- Merged rgb point clouds in real time with OpenGL
- Create mesh of point clouds with pcl http://pointclouds.org/

# SocketCommunication

### Features
- TCP server socket
- recieves binary rgb point cloud data and stores it in ./PCD/ as a timestamp.PCD file

Is used for sending binary rgb PCD data via socket in RGBPointCloudBuilder application

usage: ./socket port

# Cloud2Mesh

This project is based on the samples from the Google Tango API. You can find it at https://github.com/googlesamples/tango-examples-java

### Features
**Learn Area**
- Learn an area with area learning.
- Store an ADF file.

**Record Point Clouds**
- Load ADF file.
- Wait for localization and use device pose w.r.t to ADF file.
- Store point clouds as ascii *.PCD file.

### Todo
- Store point clouds as binary *.PCD file to reduce file size.

**Send Point Clouds**
- Socket to send point cloud *.PCD files to a server
- Create mesh of point clouds with pcl http://pointclouds.org/
