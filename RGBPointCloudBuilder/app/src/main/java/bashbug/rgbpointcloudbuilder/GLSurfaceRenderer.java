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

import android.opengl.GLSurfaceView;
import android.util.Log;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * GLSurfaceRenderer renders graphic content.
 */
public class GLSurfaceRenderer implements GLSurfaceView.Renderer {

    private MainActivity mMainActivity;
    private long timeStart;
    private long timeEnd;
    private long timeDelta;

    public GLSurfaceRenderer(MainActivity mainActivity) {
        mMainActivity = mainActivity;
    }

    public void onDrawFrame(GL10 glUnused) {
        timeStart = System.currentTimeMillis();
        JNIInterface.render();
        /*timeEnd = System.currentTimeMillis();
        timeDelta = timeEnd - timeStart;
        if (timeDelta < 20) {
            try {
                Thread.sleep(20 - timeDelta);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }*/
    }

    public void onSurfaceChanged(GL10 glUnused, int width, int height) {
        JNIInterface.setViewPort(width, height);
    }

    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        Log.e("GLSurfaceRenderer", "onSurfaceCreated");
        mMainActivity.surfaceCreated();
    }
}
