/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define GLM_FORCE_RADIANS

#include <jni.h>

#include "rgb-depth-sync/rgb_depth_sync_application.h"

static rgb_depth_sync::SynchronizationApplication app;

#ifdef __cplusplus
extern "C" {
#endif
JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_tangoInitialize(
    JNIEnv* env, jobject, jobject activity) {
  JavaVM* javaVM;
  env->GetJavaVM(&javaVM);
  return app.TangoInitialize(env, activity, javaVM);
}

JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_startPCDWorker(
    JNIEnv*, jobject) {
  app.StartPCDWorker();
}

JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_stopPCDWorker(
    JNIEnv*, jobject) {
  app.StopPCDWorker();
}

JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_tangoSetupConfig(
    JNIEnv*, jobject) {
  return app.TangoSetupConfig();
}

JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_tangoConnectCallbacks(
    JNIEnv*, jobject) {
  return app.TangoConnectCallbacks();
}

JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_tangoConnect(
    JNIEnv*, jobject) {
  return app.TangoConnect();
}

JNIEXPORT jint JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_tangoSetIntrinsicsAndExtrinsics(
    JNIEnv*, jobject) {
  return app.TangoSetIntrinsicsAndExtrinsics();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_tangoDisconnect(
    JNIEnv*, jobject) {
  app.TangoDisconnect();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_initializeGLContent(
    JNIEnv*, jobject) {
  app.InitializeGLContent();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_setViewPort(
    JNIEnv*, jobject, jint width, jint height) {
  app.SetViewPort(width, height);
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_setRangeValue(
    JNIEnv*, jobject, jfloat range) {
  app.SetRangeValue(range);
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_render(JNIEnv*, jobject) {
  app.Render();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_freeGLContent(
    JNIEnv*, jobject) {
  app.FreeGLContent();
}

std::string ConvertJString(JNIEnv* env, jstring str)  {
  //if ( !str ) LString();
  const jsize len = env->GetStringUTFLength(str);
  const char* strChars = env->GetStringUTFChars(str, (jboolean *)0);
  std::string Result(strChars, len);
  env->ReleaseStringUTFChars(str, strChars);
  return Result;
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_onTouchEvent(
    JNIEnv*, jobject, int touch_count, int event, float x0, float y0, float x1, float y1) {
  using namespace tango_gl;
  GestureCamera::TouchEvent touch_event = static_cast<GestureCamera::TouchEvent>(event);
  app.OnTouchEvent(touch_count, touch_event, x0, y0, x1, y1);
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_setCamera(
    JNIEnv*, jobject, int camera_index) {
  using namespace tango_gl;
  GestureCamera::CameraType cam_type = static_cast<GestureCamera::CameraType>(camera_index);
  app.SetCameraType(cam_type);
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_showSMMesh(
    JNIEnv*, jobject) {
  app.ShowSMMesh();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_showMSMMesh(
    JNIEnv*, jobject) {
  app.ShowMSMMesh();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_showUnOPTMesh(
    JNIEnv*, jobject) {
app.ShowUnOPTMesh();
}

JNIEXPORT void JNICALL
Java_bashbug_rgbpointcloudbuilder_JNIInterface_optimizeAndSaveToFolder(
    JNIEnv* env, jobject, jstring folder_name) {
std::string folder_name_s = ConvertJString(env, folder_name);
app.OptimizeAndSaveToFolder(folder_name_s);
}

#ifdef __cplusplus
}
#endif
