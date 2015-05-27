cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pcdFileReader)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# add the executable
add_executable(pcl_to_vtk pclToMesh.cpp)

#link installed & local libraries
target_link_libraries(pcl_to_vtk ${PCL_LIBRARIES} ${VTK_LIBRARIES})
