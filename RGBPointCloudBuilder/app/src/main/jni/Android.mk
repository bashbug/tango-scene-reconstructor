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

### tango client api
include $(CLEAR_VARS)
LOCAL_MODULE := libtango_client_api-prebuilt
LOCAL_SRC_FILES := $(PROJECT_ROOT)/TangoSDK_Gemma_C/libtango_client_api.so
LOCAL_EXPORT_C_INCLUDES := $(PROJECT_ROOT)/TangoSDK_Gemma_C/
include $(PREBUILT_SHARED_LIBRARY)

### tango support api
include $(CLEAR_VARS)
LOCAL_MODULE := libtango_support_api-prebuilt
LOCAL_SRC_FILES := $(PROJECT_ROOT)/TangoSupport_Gemma_C/libtango_support_api.so
LOCAL_EXPORT_C_INCLUDES := $(PROJECT_ROOT)/TangoSupport_Gemma_C/
include $(PREBUILT_SHARED_LIBRARY)

### include g2o

G20ROOT := /home/anastasia/Projects/TangoProject/third-party/g2o/

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

include $(CLEAR_VARS)
LOCAL_MODULE := opencv-java3-prebuilt
LOCAL_SRC_FILES := $(PROJECT_ROOT)/third-party/OpenCV-android-nonfree/libs/armeabi-v7a/libopencv_java3.so
LOCAL_EXPORT_C_INCLUDES := $(PROJECT_ROOT)/third-party/OpenCV-android-sdk/sdk/native/jni/include/
include $(PREBUILT_SHARED_LIBRARY)

### include opencv

CVROOT := $(PROJECT_ROOT)/third-party/OpenCV-android-sdk/sdk/native/jni

include $(CLEAR_VARS)
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=STATIC
include $(CVROOT)/OpenCV.mk

LOCAL_ARM_NEON := true

LOCAL_MODULE    += librgb_point_cloud_builder

LOCAL_STATIC_LIBRARIES += flann
LOCAL_STATIC_LIBRARIES += flann_cpp

LOCAL_SHARED_LIBRARIES += libtango_client_api-prebuilt
LOCAL_SHARED_LIBRARIES += libtango_support_api-prebuilt

#LOCAL_SHARED_LIBRARIES += opencv-libnonfree-prebuilt
#LOCAL_SHARED_LIBRARIES += opencv-java3-prebuilt

LOCAL_SHARED_LIBRARIES += libg2o_core-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_stuff-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_types_slam3d-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_ext_csparse-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_csparse_extension-prebuilt
LOCAL_SHARED_LIBRARIES += libg2o_solver_csparse-prebuilt

LOCAL_CFLAGS    += -std=c++11 -frtti -fexceptions

LOCAL_C_INCLUDES += $(PROJECT_ROOT)/tango-gl/include \
                    $(PROJECT_ROOT)/third-party/glm/ \
                    $(PROJECT_ROOT)/third-party/Eigen/ \
                    $(PROJECT_ROOT)/third-party/g2o/ \
                    $(PROJECT_ROOT)/third-party/projective-scan-matcher-3d/

LOCAL_SRC_FILES += conversion.cc \
                   img_file_writer.cc \
                   jni_interface.cc \
                   loop_closure_detector.cc \
                   mesh.cc \
                   pcd.cc \
                   pcd_container.cc \
                   pcd_file_reader.cc \
                   pcd_file_writer.cc \
                   pcd_worker.cc \
                   rgb_depth_sync_application.cc \
                   scan_matcher.cc \
                   shader.cc \
                   slam3d.cc \
                   scene.cc \
                   tcp_client.cc \
                   texture.cc \
                   texture_drawable.cc \
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
                   $(PROJECT_ROOT_FROM_JNI)/third-party/projective-scan-matcher-3d/projectiveScanMatcher3d/projectiveScanMatcher3d.cpp

LOCAL_LDLIBS    += -llog -lGLESv2 -L$(SYSROOT)/usr/lib

include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT))