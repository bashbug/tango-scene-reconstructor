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
import android.app.FragmentManager;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.graphics.Point;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.os.StrictMode;
import android.util.Log;
import android.view.Display;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Calendar;

/**
 * Activity that load up the main screen of the app, this is the launcher activity.
 */
public class MainActivity extends Activity implements View.OnClickListener {
    // Used for startActivityForResult on our motion tracking permission.
    private static final int REQUEST_PERMISSION_MOTION_TRACKING = 0;
    /// The input argument is invalid.
    private static final int  TANGO_INVALID = -2;
    /// This error code denotes some sort of hard error occurred.
    private static final int  TANGO_ERROR = -1;
    /// This code indicates success.
    private static final int  TANGO_SUCCESS = 0;

    // Motion Tracking permission request action.
    private static final String MOTION_TRACKING_PERMISSION_ACTION =
            "android.intent.action.REQUEST_TANGO_PERMISSION";

    // Key string for requesting and checking Motion Tracking permission.
    private static final String MOTION_TRACKING_PERMISSION =
            "MOTION_TRACKING_PERMISSION";

    private GLSurfaceRenderer mRenderer;
    private GLSurfaceView mGLView;

    private Boolean mTangoResumed = false;
    private Boolean mTangoPaused = false;
    private Boolean mTangoPausedResumedNewSurface = false;

    private ZipFiles mZipFiles;

    private Point mScreenSize;

    File mFileDirectionPCD, mFileDirectionPPM, mFileDirectionPCD_opt, mFileDirectionPCD_opt_mf, mFileDirectionGraph;
    String mFolderName;

    private boolean mIsConnectedService = false;

    private ToggleButton mStartAndStopButton;
    private SeekBar mRangeSeekbar;

    private CheckBox mSMMeshCheckbox;
    private CheckBox mMSMMeshCheckbox;
    private CheckBox mUnOPTMeshCheckbox;

    private static final String TAG = "RGBDepthSync";

    private class RangeSeekbarListener implements SeekBar.OnSeekBarChangeListener {
        int minimumValue = 200;

        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            int progressChanged = minimumValue + progress;
            JNIInterface.setRangeValue((float) progressChanged / (float) seekBar.getMax());
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {}

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {}
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        Display display = getWindowManager().getDefaultDisplay();
        mScreenSize = new Point();
        display.getSize(mScreenSize);

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);

        int status = JNIInterface.tangoInitialize(this);
        if (status != TANGO_SUCCESS) {
          if (status == TANGO_INVALID) {
            Toast.makeText(this,
              "Tango Service version mis-match", Toast.LENGTH_SHORT).show();
          } else {
            Toast.makeText(this,
              "Tango Service initialize internal error",
              Toast.LENGTH_SHORT).show();
          }
        }

        // Check if the system supports OpenGL ES 2.0.
        /*final ActivityManager activityManager = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);
        final ConfigurationInfo configurationInfo = activityManager.getDeviceConfigurationInfo();
        final boolean supportsEs2 = configurationInfo.reqGlEsVersion >= 0x20000;

        if (supportsEs2){
            Log.e(TAG, "SUPPORT OpenGL ES 2.0");
        }
        Log.e(TAG, "Version " + configurationInfo.getGlEsVersion() );*/

        setContentView(R.layout.activity_main);

        // OpenGL view where all of the graphics are drawn
        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
        // Save view before save pcd context change to show it after resume
        mGLView.setPreserveEGLContextOnPause(true);
        // Configure OpenGL renderer
        mGLView.setEGLContextClientVersion(2);
        mRenderer = new GLSurfaceRenderer(this);
        mGLView.setRenderer(mRenderer);

        // Buttons for selecting camera view and Set up button click listeners.
        findViewById(R.id.first_person_button).setOnClickListener(this);
        findViewById(R.id.third_person_button).setOnClickListener(this);
        //findViewById(R.id.top_down_button).setOnClickListener(this);

        mStartAndStopButton = (ToggleButton) findViewById(R.id.start_stop_button);

        findViewById(R.id.save_pcd_button).setOnClickListener(this);

        // start and stop recording point clouds
        //mStartStopButton = (RadioButton) findViewById(R.id.start_stop_button);
        /*mStartStopButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (mStartStopButton.isChecked()) {
                    JNIInterface.startPCD(true);
                } else {
                    JNIInterface.stopPCD(true);
                }
            }
        });*/

        // Button to start the pose optimization thread
        findViewById(R.id.optimize_pose_graph_button).setOnClickListener(this);

        // Make sure that the directories exists before saving files. Otherwise it will
        // throw an exception
        /*mFileDirectionPCD = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "./RGBPointCloudBuilder/PCD");

        mFileDirectionPCD_opt = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "./RGBPointCloudBuilder/PCD_opt");

        mFileDirectionPCD_opt_mf = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "./RGBPointCloudBuilder/PCD_opt_mf");

        mFileDirectionPPM = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "./RGBPointCloudBuilder/PPM");

        mFileDirectionGraph = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "./RGBPointCloudBuilder/Graph");

        if (mFileDirectionPCD.isDirectory()) {
            String deleteCmd = "rm -r " + mFileDirectionPCD;
            Runtime runtime = Runtime.getRuntime();
            try {
                runtime.exec(deleteCmd);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (mFileDirectionPCD_opt.isDirectory()) {
            String deleteCmd = "rm -r " + mFileDirectionPCD_opt;
            Runtime runtime = Runtime.getRuntime();
            try {
                runtime.exec(deleteCmd);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (mFileDirectionPPM.isDirectory()) {
            String deleteCmd = "rm -r " + mFileDirectionPPM;
            Runtime runtime = Runtime.getRuntime();
            try {
                runtime.exec(deleteCmd);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if(!mFileDirectionPCD.isDirectory()) {
            mFileDirectionPCD.mkdirs();
            Log.i(TAG, "RGBPointCloudBuilder/PCD Directory not exists");
        }
        if(!mFileDirectionPCD_opt.isDirectory()) {
            mFileDirectionPCD_opt.mkdirs();
            Log.i(TAG, "RGBPointCloudBuilder/PCD_opt Directory not exists");
        }
        if(!mFileDirectionPCD_opt_mf.isDirectory()) {
            mFileDirectionPCD_opt_mf.mkdirs();
            Log.i(TAG, "RGBPointCloudBuilder/PCD_opt_mf Directory not exists");
        }
        if(!mFileDirectionPPM.isDirectory()) {
            mFileDirectionPPM.mkdirs();
            Log.i(TAG, "RGBPointCloudBuilder/PPM Directory not exists");
        }
        if(!mFileDirectionGraph.isDirectory()) {
            mFileDirectionGraph.mkdirs();
            Log.i(TAG, "RGBPointCloudBuilder/Graph Directory not exists");
        }
        if (!mFileDirectionPCD.isDirectory()) {
            Log.i(TAG, "RGBPointCloudBuilder/PCD Directory not created");
        }
        if (!mFileDirectionPCD_opt.isDirectory()) {
            Log.i(TAG, "RGBPointCloudBuilder/PCD_opt Directory not created");
        }
        if (!mFileDirectionPCD_opt_mf.isDirectory()) {
            Log.i(TAG, "RGBPointCloudBuilder/PCD_opt_mf Directory not created");
        }
        if (!mFileDirectionPPM.isDirectory()) {
            Log.i(TAG, "RGBPointCloudBuilder/PPM Directory not created");
        }
        if (!mFileDirectionGraph.isDirectory()) {
            Log.i(TAG, "RGBPointCloudBuilder/Graph Directory not created");
        }*/

        mRangeSeekbar = (SeekBar) findViewById(R.id.range_seekbar);
        mRangeSeekbar.setOnSeekBarChangeListener(new RangeSeekbarListener());

        mSMMeshCheckbox = (CheckBox) findViewById(R.id.single_frame_opt_checkbox);
        mSMMeshCheckbox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton view, boolean isChecked) {
                if (isChecked) {
                    mMSMMeshCheckbox.setChecked(false);
                    mUnOPTMeshCheckbox.setChecked(false);
                    JNIInterface.showSMMesh();
                } else {
                    mUnOPTMeshCheckbox.setChecked(true);
                }
            }
        });

        mMSMMeshCheckbox = (CheckBox) findViewById(R.id.multi_frame_opt_checkbox);
        mMSMMeshCheckbox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton view, boolean isChecked) {
                if (isChecked) {
                    mSMMeshCheckbox.setChecked(false);
                    mUnOPTMeshCheckbox.setChecked(false);
                    JNIInterface.showMSMMesh();
                } else {
                    mUnOPTMeshCheckbox.setChecked(true);
                }
            }
        });

        mUnOPTMeshCheckbox = (CheckBox) findViewById(R.id.unopt_checkbox);
        mUnOPTMeshCheckbox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton view, boolean isChecked) {
                if (isChecked) {
                    mSMMeshCheckbox.setChecked(false);
                    mMSMMeshCheckbox.setChecked(false);
                    JNIInterface.showUnOPTMesh();
                }
            }
        });

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.ic_file_upload:
                ZipAndShare();
                return true;
            default:
                // If we got here, the user's action was not recognized.
                // Invoke the superclass to handle it.
                return super.onOptionsItemSelected(item);
        }
    }

    private void ZipAndShare() {
        final Intent intent = new Intent(Intent.ACTION_SEND);
        intent.setType("*/*");
        ZipFiles zipFiles = new ZipFiles();
        String zipName = zipFiles.setFolder(mFolderName);
        File f = new File(zipName);

        Uri uri = Uri.fromFile(f);
        if (uri != null) {
            Log.e(TAG, "Attache file: " + uri.toString());
            intent.putExtra(Intent.EXTRA_STREAM, uri);
        } else {
            Log.e(TAG, "No file " + uri.toString() + "found!");
        }

        startActivity(Intent.createChooser(intent, "Choose client"));
    }

    public void onToggleClick(View v) {
        if (mStartAndStopButton.isChecked()) {
            Log.e(TAG, "is checked");
            if (mTangoPausedResumedNewSurface) {
                //JNIInterface.freeGLContent();
                //JNIInterface.initializeGLContent();
                mTangoPausedResumedNewSurface = false;
            }
            JNIInterface.startPCDWorker();
        } else {
            JNIInterface.stopPCDWorker();
            Log.e(TAG, "is NOT checked");
        }
    }

    @Override
    public void onClick(View v) {

        switch (v.getId()) {
            case R.id.first_person_button:
                JNIInterface.setCamera(0);
                break;
            case R.id.third_person_button:
                JNIInterface.setCamera(1);
                break;
            case R.id.optimize_pose_graph_button:
                JNIInterface.setCamera(1);
                String date = new SimpleDateFormat("yyyyMMddHHmmss").format(Calendar.getInstance().getTime());
                mFolderName = Environment.getExternalStorageDirectory().toString() + "/Documents/RGBPointCloudBuilder/" + date;
                Log.e(TAG, mFolderName+"/");
                JNIInterface.optimizeAndSaveToFolder(mFolderName+"/");
                break;
            case R.id.save_pcd_button:
                //JNIInterface.savePCD(true);
                break;
            default:
                Log.w(TAG, "Unrecognized button click.");
                return;
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        // Pass the touch event to the native layer for camera control.
        // Single touch to rotate the camera around the device.
        // Two fingers to zoom in and out.
        int pointCount = event.getPointerCount();
        if (pointCount == 1) {
            float normalizedX = event.getX(0) / mScreenSize.x;
            float normalizedY = event.getY(0) / mScreenSize.y;
            JNIInterface.onTouchEvent(1,
                    event.getActionMasked(), normalizedX, normalizedY, 0.0f, 0.0f);
        }
        if (pointCount == 2) {
            if (event.getActionMasked() == MotionEvent.ACTION_POINTER_UP) {
                int index = event.getActionIndex() == 0 ? 1 : 0;
                float normalizedX = event.getX(index) / mScreenSize.x;
                float normalizedY = event.getY(index) / mScreenSize.y;
                JNIInterface.onTouchEvent(1,
                        MotionEvent.ACTION_DOWN, normalizedX, normalizedY, 0.0f, 0.0f);
            } else {
                float normalizedX0 = event.getX(0) / mScreenSize.x;
                float normalizedY0 = event.getY(0) / mScreenSize.y;
                float normalizedX1 = event.getX(1) / mScreenSize.x;
                float normalizedY1 = event.getY(1) / mScreenSize.y;
                JNIInterface.onTouchEvent(2, event.getActionMasked(),
                        normalizedX0, normalizedY0, normalizedX1, normalizedY1);
            }
        }
        return true;
    }

    @Override
    protected void onResume() {
        // We moved most of the onResume lifecycle calls to the surfaceCreated,
        // surfaceCreated will be called after the GLSurface is created.
        super.onResume();
        mGLView.onResume();

        // Though we're going to use Tango's C interface so that we have more
        // low level control of our graphics, we can still use the Java API to
        // check that we have the correct permissions.
        if (!hasPermission(this, MOTION_TRACKING_PERMISSION)) {
            Log.e(TAG, "LOST PERMISSION");
            getMotionTrackingPermission();
        }
        if (mTangoPaused) {
            Log.e(TAG, "ONRESUME + ONPAUSE");
            surfaceCreated();
            mTangoPaused = false;
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        mGLView.onPause();
        if (mIsConnectedService) {
            mTangoResumed = true;
            mTangoPaused = true;
            Log.e(TAG, "TANGO IS DISCONNECTED");
            JNIInterface.tangoDisconnect();
        }
        //JNIInterface.freeGLContent();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    public void surfaceCreated() {
        Log.e(TAG, "SURFACE CREATED");

        if (!mTangoResumed) {
            JNIInterface.initializeGLContent();
        } else {
            Log.e(TAG, "WAS RESUMED");
            mTangoResumed = false;
            mTangoPausedResumedNewSurface = true;
        }

        int ret = JNIInterface.tangoSetupConfig();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to set config with code: "  + ret);
            finish();
        }

        ret = JNIInterface.tangoConnectCallbacks();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to set connect cbs with code: "  + ret);
            finish();
        }

        ret = JNIInterface.tangoConnect();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to set connect service with code: "  + ret);
            finish();
        }

        ret = JNIInterface.tangoSetIntrinsicsAndExtrinsics();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to extrinsics and intrinsics code: "  + ret);
            finish();
        }

        //JNIInterface.setRangeValue(0.000001f);

        mIsConnectedService = true;
    }

    @Override
    protected void onActivityResult (int requestCode, int resultCode, Intent data) {
        if (requestCode == REQUEST_PERMISSION_MOTION_TRACKING) {
            if (resultCode == RESULT_CANCELED) {
                mIsConnectedService = false;
                finish();
            }
        }
    }

    public boolean hasPermission(Context context, String permissionType){
        Uri uri = Uri.parse("content://com.google.atap.tango.PermissionStatusProvider/" +
                permissionType);
        Cursor cursor = context.getContentResolver().query(uri, null, null, null, null);
        if (cursor == null) {
            return false;
        } else {
            return true;
        }
    }

    // Call the permission intent for the Tango Service to ask for motion tracking
    // permissions. All permission types can be found here:
    //   https://developers.google.com/project-tango/apis/c/c-user-permissions
    private void getMotionTrackingPermission() {
        Intent intent = new Intent();
        intent.setAction(MOTION_TRACKING_PERMISSION_ACTION);
        intent.putExtra("PERMISSIONTYPE", MOTION_TRACKING_PERMISSION);

        // After the permission activity is dismissed, we will receive a callback
        // function onActivityResult() with user's result.
        startActivityForResult(intent, 0);
    }

    private void showSetServerSocketAddresssAndIPDialog() {
        FragmentManager manager = getFragmentManager();
        SetServerSocketAddressAndIPDialog setServerSocketAddressAndIPDialog = new SetServerSocketAddressAndIPDialog();
        setServerSocketAddressAndIPDialog.show(manager, "SocketAddIPDialog");
    }

    /*public void setmSendPCDSwitch(boolean on) {
        mSendPCDSwitch.setEnabled(on);
    }*/
}
