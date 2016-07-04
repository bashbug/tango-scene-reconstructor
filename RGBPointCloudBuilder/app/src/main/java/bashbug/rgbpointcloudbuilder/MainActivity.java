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
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.database.Cursor;
import android.graphics.Point;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.os.StrictMode;
import android.preference.Preference;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Display;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.FrameLayout;
import android.widget.RadioButton;
import android.widget.SeekBar;
import android.widget.TextView;
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
    private Boolean mShowOptions = false;

    private ZipFiles mZipFiles;

    private Point mScreenSize;

    File mFileDirectionPCD, mFileDirectionPPM, mFileDirectionPCD_opt, mFileDirectionPCD_opt_mf, mFileDirectionGraph;
    String mFolderName;

    private boolean mIsConnectedService = false;

    private Button mOptimizeButton;
    private ToggleButton mStartAndStopButton;
    private SeekBar mRangeSeekbar;

    private RadioButton mSMMeshRadioButton;
    private RadioButton mMSMMeshRadioButton;
    private RadioButton mUnOPTMeshRadioButton;

    MenuItem mOptions;
    SettingsFragment mSettingFragment;
    MenuItem mShareFiles;
    MenuItem mAnalytics;
    FrameLayout mAnalyticsFrameLayout;

    private int mFTFSMAverageCPUTime = 0;
    private int mFTFSMCPUTime = 0;
    private int mMFSMAverageCPUTime = 0;
    private int mMFSMCPUTime = 0;

    private TextView mFTFSMAnalycticsTextView;
    private TextView mMFSMAnalycticsTextView;

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

    // Tango Service connection.
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            JNIInterface.onTangoServiceConnected(service);

            int ret = JNIInterface.tangoSetupConfig();
            if (ret != TANGO_SUCCESS) {
                Log.e(TAG, "Failed to set config.");
                finish();
            }

            ret = JNIInterface.tangoConnectCallbacks();
            if (ret != TANGO_SUCCESS) {
                Log.e(TAG, "Failed to set connect callbacks.");
                finish();
            }

            ret = JNIInterface.tangoConnect();
            if (ret != TANGO_SUCCESS) {
                Log.e(TAG, "Failed to set connect service.");
                finish();
            }

            ret = JNIInterface.tangoSetIntrinsicsAndExtrinsics();
            Log.e(TAG, "JNIInterface.tangoSetIntrinsicsAndExtrinsics()");
            if (ret != TANGO_SUCCESS) {
                Log.e(TAG, "Failed to set extrinsics and intrinsics.");
                finish();
            }

            mIsConnectedService = true;
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    public void setComputationTimes(int avrgCPUTimeFTFSM, int CPUTimeFTFSM, int avrgCPUTimeMFSM, int CPUTimeMFSM) {
        mFTFSMAverageCPUTime = avrgCPUTimeFTFSM;
        mFTFSMCPUTime = CPUTimeFTFSM;
        mMFSMAverageCPUTime = avrgCPUTimeMFSM;
        mMFSMCPUTime = CPUTimeMFSM;
        mFTFSMAnalycticsTextView.setText("Ø " + Integer.toString(mFTFSMAverageCPUTime) + " ms/frame " +
                Integer.toString(mFTFSMCPUTime) + " ms cpu time");
        mMFSMAnalycticsTextView.setText("Ø " + Integer.toString(mMFSMAverageCPUTime) + " ms/iter "
                + Integer.toString(mMFSMCPUTime) + " ms cpu time");
    }

    public void setFTFInfo(int loop_closure, int sm_frames, int frames) {
        TextView t = (TextView) findViewById(R.id.mfsm_analytics_2);
        t.setText(Integer.toString(loop_closure) + " good/" + Integer.toString(sm_frames)
                + " loops of " + Integer.toString(frames) + " frames");
    }

    public void setCheckBoxesVisibleAndEnableButtons() {
        mUnOPTMeshRadioButton.setVisibility(View.VISIBLE);
        mSMMeshRadioButton.setVisibility(View.VISIBLE);
        mMSMMeshRadioButton.setVisibility(View.VISIBLE);
        mShareFiles.setEnabled(true);
        mShareFiles.getIcon().setAlpha(255);
        mAnalytics.setEnabled(true);
        mAnalytics.getIcon().setAlpha(255);
    }

    public void setCheckBoxesInVisibleAndDisableButtons() {
        mUnOPTMeshRadioButton.setVisibility(View.GONE);
        mSMMeshRadioButton.setVisibility(View.GONE);
        mMSMMeshRadioButton.setVisibility(View.GONE);
        mShareFiles.setEnabled(false);
        mShareFiles.getIcon().setAlpha(130);
        mAnalytics.setEnabled(false);
        mAnalytics.getIcon().setAlpha(130);
    }

    public void toggleAnalytics() {
        if (mAnalyticsFrameLayout.getVisibility() == View.GONE) {
            mAnalyticsFrameLayout.setVisibility(View.VISIBLE);
        } else {
            mAnalyticsFrameLayout.setVisibility(View.GONE);
        }
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
        findViewById(R.id.third_person_button).setEnabled(false);
        //findViewById(R.id.top_down_button).setOnClickListener(this);

        mStartAndStopButton = (ToggleButton) findViewById(R.id.start_stop_button);

        mFTFSMAnalycticsTextView = (TextView) findViewById(R.id.ftfsm_analytics);
        mMFSMAnalycticsTextView = (TextView) findViewById(R.id.mfsm_analytics);

        mSettingFragment = new SettingsFragment();

        //findViewById(R.id.save_pcd_button).setOnClickListener(this);

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

        mAnalyticsFrameLayout = (FrameLayout) findViewById(R.id.analytics);
        mAnalyticsFrameLayout.setVisibility(View.GONE);

        // Button to start the pose optimization thread
        mOptimizeButton = (Button) findViewById(R.id.optimize_pose_graph_button);
        mOptimizeButton.setEnabled(false);
        mOptimizeButton.setOnClickListener(this);

        mRangeSeekbar = (SeekBar) findViewById(R.id.range_seekbar);
        mRangeSeekbar.setOnSeekBarChangeListener(new RangeSeekbarListener());

        mSMMeshRadioButton = (RadioButton) findViewById(R.id.ftfsm_mesh_radio_button);
        mSMMeshRadioButton.setVisibility(View.GONE);

        mMSMMeshRadioButton = (RadioButton) findViewById(R.id.mfsm_mesh_radio_button);
        mMSMMeshRadioButton.setVisibility(View.GONE);

        mUnOPTMeshRadioButton = (RadioButton) findViewById(R.id.tango_mesh_radio_button);
        mUnOPTMeshRadioButton.setVisibility(View.GONE);
    }

    public void onRadioButtonClicked(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        switch(view.getId()) {
            case R.id.ftfsm_mesh_radio_button:
                if (checked)
                    JNIInterface.showSMMesh();
                break;
            case R.id.mfsm_mesh_radio_button:
                if (checked)
                    JNIInterface.showMSMMesh();
                break;
            case R.id.tango_mesh_radio_button:
                if (checked)
                    JNIInterface.showUnOPTMesh();
                break;
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);
        mShareFiles = menu.findItem(R.id.ic_file_upload);
        mShareFiles.setEnabled(false);
        mShareFiles.getIcon().setAlpha(130);

        mAnalytics = menu.findItem(R.id.ic_poll);
        mAnalytics.setEnabled(false);
        mAnalytics.getIcon().setAlpha(130);

        mOptions = menu.findItem(R.id.ic_options);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.ic_file_upload:
                ZipAndShare();
                Toast.makeText(this, "ZIP file is also saved to Documents/RGBPointCloudBuilder", Toast.LENGTH_LONG).show();
                return true;
            case R.id.ic_poll:
                toggleAnalytics();
                return true;
            case R.id.ic_options:
                if (mShowOptions) {
                    // Display the fragment as the main content.
                    getFragmentManager().beginTransaction()
                            .remove(mSettingFragment)
                            .commit();
                    mShowOptions = false;
                } else {
                    // Display the fragment as the main content.
                    getFragmentManager().beginTransaction()
                            .replace(android.R.id.content, mSettingFragment)
                            .commit();
                    mShowOptions = true;
                }
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
            mStartAndStopButton.setAlpha(1.0f);
            mOptimizeButton.setEnabled(false);
            JNIInterface.setCamera(0);
            findViewById(R.id.third_person_button).setEnabled(false);
            setCheckBoxesInVisibleAndDisableButtons();
            Log.e(TAG, "is checked");
            if (mTangoPausedResumedNewSurface) {
                //JNIInterface.freeGLContent();
                //JNIInterface.initializeGLContent();
                mTangoPausedResumedNewSurface = false;
            }
            JNIInterface.startPCDWorker();
        } else {
            mStartAndStopButton.setAlpha(0.8f);
            mOptimizeButton.setEnabled(true);
            findViewById(R.id.third_person_button).setEnabled(true);
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
                JNIInterface.optimizeAndSaveToFolder(mFolderName + "/");
                setCheckBoxesVisibleAndEnableButtons();
                Toast.makeText(this, "PCD files are saved to Documents/RGBPointCloudBuilder", Toast.LENGTH_LONG).show();
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

        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);

        /*// Though we're going to use Tango's C interface so that we have more
        // low level control of our graphics, we can still use the Java API to
        // check that we have the correct permissions.
        if (!hasPermission(this, MOTION_TRACKING_PERMISSION)) {
            Log.e(TAG, "LOST PERMISSION");
            getMotionTrackingPermission();
        }*/
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
            unbindService(mTangoServiceConnection);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    public void surfaceCreated() {
        Log.e(TAG, "SURFACE CREATED");

        if (!mTangoResumed) {
            JNIInterface.initializeGLContent();
            setPreferences();
        } else {
            Log.e(TAG, "WAS RESUMED");
            mTangoResumed = false;
            mTangoPausedResumedNewSurface = true;
        }

        /*int ret = JNIInterface.tangoSetupConfig();
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

        mIsConnectedService = true;*/
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

    private void setPreferences() {
        SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
        boolean value = sharedPref.getBoolean("background_color", true);
        JNIInterface.setBackgroundColorBlack(value);
        value = sharedPref.getBoolean("grid", true);
        JNIInterface.setGridOn(value);
        value =  sharedPref.getBoolean("ftfsm", true);
        boolean value2 = sharedPref.getBoolean("mfsm", true);

        if (value && !value2) {
            JNIInterface.setOptimizationMethods(0);
        } else if (!value && value2) {
            JNIInterface.setOptimizationMethods(1);
        } else {
            JNIInterface.setOptimizationMethods(2);
        }

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
