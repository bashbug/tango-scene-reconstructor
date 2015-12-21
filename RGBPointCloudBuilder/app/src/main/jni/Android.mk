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

#PCL_INCLUDE := $(PROJECT_ROOT)/third-party/pcl
#BOOST_INCLUDE := $(PROJECT_ROOT)/third-party/boost

include $(CLEAR_VARS)
LOCAL_MODULE    := librgb_point_cloud_builder

LOCAL_SHARED_LIBRARIES := tango_client_api
LOCAL_CFLAGS    := -std=c++11 -frtti

LOCAL_C_INCLUDES := $(PROJECT_ROOT)/tango-service-sdk/include/ \
                    $(PROJECT_ROOT)/tango-gl/include \
                    $(PROJECT_ROOT)/third-party/glm/ \
                    $(PROJECT_ROOT)/third-party/Eigen/ \
                    $(PROJECT_ROOT)/third-party/projective-scan-matcher-3d/ \

                    # $(PROJECT_ROOT)/third-party/pcl/ \
                    # $(PROJECT_ROOT)/third-party/boost \
                    # $(PROJECT_ROOT)/third-party/eigen3

LOCAL_SRC_FILES := shader.cc \
                   texture.cc \
                   texture_drawable.cc \
                   color_image.cc \
                   range_image.cc \
                   jni_interface.cc \
                   mesh.cc \
                   point.cc \
                   point_cloud_data.cc \
                   point_cloud_container.cc \
                   pose_container.cc \
                   rgb_depth_sync_application.cc \
                   scene.cc \
                   tcp_client.cc \
                   util.cc \
                   write_color_image.cc \
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
                   $(PROJECT_ROOT_FROM_JNI)/third-party/projective-scan-matcher-3d/projectiveScanMatcher3d/projectiveScanMatcher3d.cpp
                   
LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib
# LOCAL_LDLIBS    := -llog -lGLESv2 -L$(SYSROOT)/usr/lib -L$(PCL_INCLUDE)/lib -L$(BOOST_INCLUDE)/lib
include $(BUILD_SHARED_LIBRARY)
$(call import-add-path, $(PROJECT_ROOT))
$(call import-module,tango_client_api)
