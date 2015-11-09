# RGB Point Cloud Builder

This project is based on the samples from the Google Tango API. You can find it at https://github.com/googlesamples/tango-examples-c

### Features
**Sync image with point cloud**
- Using pose of image and point cloud timestamp.
- Store rgb point cloud as timestamp.PCD binary file.

**Send Point Clouds**
- Socket to send point cloud *.PCD files to a server (uses SocketCommunication/socket.cpp)

### Todo
**visualize with OpenGL**
- Merged rgb point clouds in real time with OpenGL
- Create mesh of point clouds with pcl http://pointclouds.org/

# SocketCommunication

Is used for sending binary rgb PCD data of via socket
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
