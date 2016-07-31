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

package bashbug.tangoscenereconstructor;

import android.app.Activity;
import android.util.Log;
import android.os.IBinder;

/**
 * Interfaces between C and Java.
 */
public class JNIInterface {

    static {
        // This project depends on tango_client_api, so we need to make sure we load
        // the correct library first.
        if (TangoInitializationHelper.loadTangoSharedLibrary() ==
                TangoInitializationHelper.ARCH_ERROR) {
            Log.e("TangoJNINative", "ERROR! Unable to load libtango_client_api.so!");
        }
        System.loadLibrary("tango_scene_reconstructor");
    }

    public static native void onTangoServiceConnected(IBinder binder);

    public static native int tangoInitialize(Activity activity);

    public static native int tangoSetupConfig();

    public static native int tangoConnect();

    public static native int tangoConnectCallbacks();

    public static native int tangoSetIntrinsicsAndExtrinsics();

    public static native void tangoDisconnect();

    public static native void initializeGLContent();

    public static native void freeGLContent();

    public static native void setViewPort(int width, int height);

    public static native void render();

    public static native void startPCDWorker();

    public static native void stopPCDWorker();

    public static native void setRangeValue(float range);

    // Pass touch events to the native layer.
    public static native void onTouchEvent(int touchCount, int event0,
                                           float x0, float y0, float x1, float y1);
    // Set the render camera's viewing angle:
    //   first person, third person, or top down.
    public static native void setCamera(int cameraIndex);

    public static native void showSMMesh();

    public static native void showMSMMesh();

    public static native void showUnOPTMesh();

    public static native void optimizeAndSaveToFolder(String folder_name);

    public static native void setBackgroundColorBlack(boolean on);

    public static native void setGridOn(boolean on);

    public static native void setOptimizationMethods(int opt);


}
