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

package bashbug.rgbpointcloudbuilder;

import android.app.Activity;

/**
 * Interfaces between C and Java.
 */
public class JNIInterface {
    static {
      System.loadLibrary("rgb_point_cloud_builder");
    }

    public static native int tangoInitialize(Activity activity);

    public static native int tangoSetupConfig();

    // Set point cloud record true, if record switch button is checked
    public static native int setPCDSave(boolean isChecked);

    public static native int setPCDSend(boolean isChecked);

    public static native int storeImage(boolean store);

    public static native int tangoConnectTexture();

    public static native int tangoConnect();

    public static native int tangoConnectCallbacks();

    public static native int tangoSetIntrinsicsAndExtrinsics();

    public static native void tangoDisconnect();

    public static native void initializeGLContent();

    public static native void freeGLContent();

    public static native void setViewPort(int width, int height);

    public static native void render();

    public static native void setDepthMap(boolean on);

    public static native void setRGBMap(boolean on);

    public static native void setSocket(String addr, int port);

}
