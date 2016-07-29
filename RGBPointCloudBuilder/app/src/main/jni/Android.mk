#
# Copyright 2014 Google Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../../..
PROJECT_ROOT:= $(call my-dir)/../../../../..

### include flann
FLANN_ROOT := $(PROJECT_ROOT)/third-party/flann-android/

include $(CLEAR_VARS)
LOCAL_MODULE := libflann_cpp_s-static
LOCAL_SRC_FILES := $(FLANN_ROOT)/lib/libflann_cpp_s.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libflann_s-static
LOCAL_SRC_FILES := $(FLANN_ROOT)/lib/libflann_s.a
include $(PREBUILT_STATIC_LIBRARY)

### include boost
BOOST_ROOT := $(PROJECT_ROOT)/third-party/boost-android/

include $(CLEAR_VARS)
LOCAL_MODULE := libboost_filesystem-static
LOCAL_SRC_FILES := $(BOOST_ROOT)/lib/libboost_filesystem.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libboost_system-static
LOCAL_SRC_FILES := $(BOOST_ROOT)/lib/libboost_system.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libboost_thread-static
LOCAL_SRC_FILES := $(BOOST_ROOT)/lib/libboost_thread.a
include $(PREBUILT_STATIC_LIBRARY)

### include pcl
PCL_ROOT := $(PROJECT_ROOT)/third-party/pcl-android/

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_io-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_io.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_io_ply-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_io_ply.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_common-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_common.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_kdtree-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_kdtree.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_search-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_search.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_filters-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_filters.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_sample_consensus-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_sample_consensus.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_registration-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_registration.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libpcl_features-static
LOCAL_SRC_FILES := $(PCL_ROOT)/lib/libpcl_features.a
include $(PREBUILT_STATIC_LIBRARY)

### include g2o

G20ROOT := $(PROJECT_ROOT)/third-party/g2o/

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_core-prebuilt
LOCAL_SRC_FILES := $(G20ROOT)/lib/libg2o_core.so
LOCAL_EXPORT_C_INCLUDES := $(G20ROOT)/g2o/
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_stuff-prebuilt
LOCAL_SRC_FILES := $(G20ROOT)/lib/libg2o_stuff.so
LOCAL_EXPORT_C_INCLUDES := $(G20ROOT)/g2o/
include $(PREBUILT_SHARED_LIBRARY)

### g2o 3d types

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_types_slam3d-prebuilt
LOCAL_SRC_FILES := $(G20ROOT)/lib/libg2o_types_slam3d.so
LOCAL_EXPORT_C_INCLUDES := $(G20ROOT)/g2o/
include $(PREBUILT_SHARED_LIBRARY)

### g2o solver

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_ext_csparse-prebuilt
LOCAL_SRC_FILES := $(G20ROOT)/lib/libg2o_ext_csparse.so
LOCAL_EXPORT_C_INCLUDES := $(G20ROOT)/g2o/
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_csparse_extension-prebuilt
LOCAL_SRC_FILES := $(G20ROOT)/lib/libg2o_csparse_extension.so
LOCAL_EXPORT_C_INCLUDES := $(G20ROOT)/g2o/
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libg2o_solver_csparse-prebuilt
LOCAL_SRC_FILES := $(G20ROOT)/lib/libg2o_solver_csparse.so
LOCAL_EXPORT_C_INCLUDES := $(G20ROOT)/g2o/ \
                        $(G20ROOT)/EXTERNAL/csparse/
include $(PREBUILT_SHARED_LIBRARY)

### include opencv

CVROOT := $(PROJECT_ROOT)/third-party/OpenCV-android-sdk/sdk/native/jni

include $(CLEAR_VARS)
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=STATIC
include $(CVROOT)/OpenCV.mk

LOCAL_ARM_NEON := true

LOCAL_MODULE += librgb_point_cloud_builder

LOCAL_STATIC_LIBRARIES += libflann_cpp_s-static
LOCAL_STATIC_LIBRARIES += libflann_s-static

LOCAL_STATIC_LIBRARIES += libboost_filesystem-static
LOCAL_STATIC_LIBRARIES += libboost_system-static
LOCAL_STATIC_LIBRARIES += libboost_thread-static

LOCAL_STATIC_LIBRARIES += libpcl_io-static
LOCAL_STATIC_LIBRARIES += libpcl_io_ply-static
LOCAL_STATIC_LIBRARIES += libpcl_common-static
LOCAL_STATIC_LIBRARIES += libpcl_search-static
LOCAL_STATIC_LIBRARIES += libpcl_kdtree-static
LOCAL_STATIC_LIBRARIES += libpcl_filters-static
#LOCAL_STATIC_LIBRARIES += libpcl_sample_consensus-static
LOCAL_STATIC_LIBRARIES += libpcl_registration-static
LOCAL_STATIC_LIBRARIES += libpcl_features-static

LOCAL_SHARED_LIBRARIES += tango_client_api tango_support_api tango_3d_reconstruction

LOCAL_SHARED_LIBRARIES += libg2o_core-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_stuff-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_types_slam3d-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_ext_csparse-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_csparse_extension-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_solver_csparse-prebuilt

LOCAL_CFLAGS    += -std=c++11 -frtti -fexceptions -fopenmp -w

LOCAL_C_INCLUDES += $(PROJECT_ROOT)/tango-gl/include \
                    $(PROJECT_ROOT)/third-party/glm/ \
                    $(PROJECT_ROOT)/third-party/Eigen/ \
                    $(BOOST_ROOT)/include/ \
                    $(FLANN_ROOT)/include/ \
                    $(PCL_ROOT)/include/ \
                    $(PROJECT_ROOT)/third-party/g2o/ \
                    $(PROJECT_ROOT)/third-party/projective-scan-matcher-3d/ \
                    $(PROJECT_ROOT_FROM_JNI)/third-party/multiframe-scan-matcher-3d/

LOCAL_SRC_FILES += conversion.cc \
                   frame_to_frame_scan_matcher.cc \
                   jni_interface.cc \
                   mesh.cc \
                   multiframe_scan_matcher.cc \
                   pcd.cc \
                   pcd_container.cc \
                   pcd_drawable.cc \
                   pcd_outlier_removal.cc \
                   pcd_worker.cc \
                   pose_data.cc \
                   rgb_depth_sync_application.cc \
                   shader.cc \
                   scene.cc \
                   util.cc \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/axis.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/bounding_box.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/camera.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/conversions.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/cube.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/drawable_object.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/frustum.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/gesture_camera.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/grid.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/line.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/mesh.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/shaders.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/trace.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/transform.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/tango-gl/util.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/third-party/projective-scan-matcher-3d/projectiveScanMatcher3d/projectiveScanMatcher3d.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/third-party/multiframe-scan-matcher-3d/g2oEdgeTypeGicp.cpp \
                   $(PROJECT_ROOT_FROM_JNI)/third-party/multiframe-scan-matcher-3d/multiFrameIcp.cpp \

LOCAL_LDLIBS += -llog -lGLESv2 -L$(SYSROOT)/usr/lib
LOCAL_LDFLAGS += -fopenmp

include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT))
$(call import-module, tango_client_api)
$(call import-module, tango_support_api)
$(call import-module, tango_3d_reconstruction)