package bashbug.cloud2mesh;

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

import android.app.FragmentManager;
import android.content.Intent;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tango.ux.TangoUx;
import com.google.atap.tango.ux.TangoUxLayout;
import com.google.atap.tango.ux.UxExceptionEvent;
import com.google.atap.tango.ux.UxExceptionEventListener;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoAreaDescriptionMetaData;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoCameraPreview;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import java.text.DecimalFormat;
import java.util.ArrayList;

import bashbug.cloud2mesh.SetADFNameDialog.SetNameCommunicator;

/**
 * Main Activity class for the Area Learning API Sample. Handles the connection to the Tango service
 * and propagation of Tango pose data to OpenGL and Layout views. OpenGL rendering logic is
 * delegated to the {@link ALRenderer} class.
 */
public class AreaLearningActivity extends BaseActivity implements View.OnClickListener,
        SetNameCommunicator {

    private static final String TAG = AreaLearningActivity.class.getSimpleName();
    private static final int SECONDS_TO_MILLI = 1000;
    private Tango mTango;
    private TangoConfig mConfig;
    private TextView mStart2DeviceTranslationTextView;
    private TextView mAdf2DeviceTranslationTextView;
    private TextView mAdf2StartTranslationTextView;
    private TextView mStart2DeviceQuatTextView;
    private TextView mAdf2DeviceQuatTextView;
    private TextView mAdf2StartQuatTextView;
    private TextView mUUIDTextView;
    private TextView mStart2DevicePoseStatusTextView;
    private TextView mAdf2DevicePoseStatusTextView;
    private TextView mAdf2StartPoseStatusTextView;
    private TextView mStart2DevicePoseCountTextView;
    private TextView mAdf2DevicePoseCountTextView;
    private TextView mAdf2StartPoseCountTextView;

    private int mStart2DevicePoseCount;
    private int mAdf2DevicePoseCount;
    private int mAdf2StartPoseCount;
    private int mStart2DevicePreviousPoseStatus;
    private int mAdf2DevicePreviousPoseStatus;
    private int mAdf2StartPreviousPoseStatus;

    private boolean mIsRelocalized;
    private String mCurrentUUID;
    private boolean mIsTangoServiceConnected;

    private Toast mRelocalizedToastView;
    private Button mSaveAdf;
    private TangoUx mTangoUx;
    private TangoUxLayout mTangoUxLayout;

    //private ALRenderer mRenderer;
    private GLSurfaceView mGLView;

    // Camera
    private TangoCameraPreview mCameraView;

    private TangoPoseData[] mPoses;
    private static final int UPDATE_INTERVAL_MS = 100;
    private static final DecimalFormat threeDec = new DecimalFormat("00.000");
    public static Object sharedLock = new Object();

    /*
     * This is an advanced way of using UX exceptions. In most cases developers can just use the in
     * built exception notifications using the Ux Exception layout. In case a developer doesn't want
     * to use the default Ux Exception notifications, he can set the UxException listener as shown
     * below.
     * In this example we are just logging all the ux exceptions to logcat, but in a real app,
     * developers should use these exceptions to contextually notify the user and help direct the
     * user in using the device in a way Tango service expects it.
     */
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {

        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE){
                Log.i(TAG, "Device lying on surface ");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS){
                Log.i(TAG, "Very few depth points in point cloud " );
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES){
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_INCOMPATIBLE_VM){
                Log.i(TAG, "Device not running on ART");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID){
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST){
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_OVER_EXPOSED){
                Log.i(TAG, "Camera Over Exposed");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_TANGO_SERVICE_NOT_RESPONDING){
                Log.i(TAG, "TangoService is not responding ");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_TANGO_UPDATE_NEEDED){
                Log.i(TAG, "Device not running on ART");
            }
            if(uxExceptionEvent.getType() == UxExceptionEvent.TYPE_UNDER_EXPOSED){
                Log.i(TAG, "Camera Under Exposed " );
            }

        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_area_learning);
        setTitle("Learn Area");

        mAdf2DeviceTranslationTextView = (TextView) findViewById(R.id.adf2devicePose);
        mStart2DeviceTranslationTextView = (TextView) findViewById(R.id.start2devicePose);
        mAdf2StartTranslationTextView = (TextView) findViewById(R.id.adf2startPose);

        mAdf2DeviceQuatTextView = (TextView) findViewById(R.id.adf2deviceQuat);
        mStart2DeviceQuatTextView = (TextView) findViewById(R.id.start2deviceQuat);
        mAdf2StartQuatTextView = (TextView) findViewById(R.id.adf2startQuat);

        mAdf2DevicePoseStatusTextView = (TextView) findViewById(R.id.adf2deviceStatus);
        mStart2DevicePoseStatusTextView = (TextView) findViewById(R.id.start2deviceStatus);
        mAdf2StartPoseStatusTextView = (TextView) findViewById(R.id.adf2startStatus);

        mAdf2DevicePoseCountTextView = (TextView) findViewById(R.id.adf2devicePosecount);
        mStart2DevicePoseCountTextView = (TextView) findViewById(R.id.start2devicePosecount);
        mAdf2StartPoseCountTextView = (TextView) findViewById(R.id.adf2startPosecount);

        // Toast for relocalized area. Shows up for 2sek
        LayoutInflater inflater = getLayoutInflater();
        View layout = inflater.inflate(R.layout.toast,
                (ViewGroup) findViewById(R.id.toast_layout_root));
        TextView text = (TextView) layout.findViewById(R.id.text);
        text.setText("Area localized");
        mRelocalizedToastView = new Toast(getApplicationContext());
        mRelocalizedToastView.setGravity(Gravity.CENTER_HORIZONTAL, 0, 0);
        mRelocalizedToastView.setDuration(Toast.LENGTH_SHORT);
        mRelocalizedToastView.setView(layout);

        // Camera view
        mCameraView = (TangoCameraPreview) findViewById(R.id.gl_camera_view);

        mSaveAdf = (Button) findViewById(R.id.saveAdf);
        mSaveAdf.setOnClickListener(this);

        mUUIDTextView = (TextView) findViewById(R.id.uuid);

        startActivityForResult(
                Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                0);
        startActivityForResult(
                Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_ADF_LOAD_SAVE),
                1);

        // Instantiate the Tango service
        mTango = new Tango(this);
        mConfig = new TangoConfig();
        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mIsRelocalized = false;
        mIsTangoServiceConnected = false;

        // Set learning mode to config.
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, false);

        mTangoUx = new TangoUx.Builder(this).build();
        mTangoUxLayout = (TangoUxLayout) findViewById(R.id.layout_tango);
        mTangoUx = new TangoUx.Builder(this).setTangoUxLayout(mTangoUxLayout).build();
        mTangoUx.setUxExceptionEventListener(mUxExceptionListener);

        mPoses = new TangoPoseData[3];
        mStart2DevicePoseCount = 0;
        mAdf2DevicePoseCount = 0;
        mAdf2StartPoseCount = 0;

        startUIThread();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == 0) {
            Log.i(TAG, "Triggered");
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission, Toast.LENGTH_LONG).show();
                finish();
                return;
            }
        } else if (requestCode == 1) {
            Log.i(TAG, "Triggered");
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission, Toast.LENGTH_LONG).show();
                finish();
            }
            connectTango();
        }
    }

    private void connectTango () {
        try {
            setTangoListeners();
        } catch (TangoErrorException e) {
            Toast.makeText(this, R.string.TangoError, Toast.LENGTH_SHORT).show();
        } catch (SecurityException e) {
            Toast.makeText(getApplicationContext(), R.string.motiontrackingpermission,
                    Toast.LENGTH_SHORT).show();
        }

        try {
            mTango.connect(mConfig);
            Log.e("connectTango", "connect");

        } catch (TangoOutOfDateException outDateEx) {
            Log.e("TangoOutOfDateException", "started...");
            if (mTangoUx != null) {
                Log.e("mTangoUx", "started...");
                mTangoUx.onTangoOutOfDate();
            }
        } catch (TangoErrorException e) {
            Log.e("TangoErrorException", "started...");
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT)
                    .show();
        }
    }

    private void setTangoListeners() {

        // Set Tango Listeners for Poses Device wrt Start of Service, Device wrt
        // ADF and Start of Service wrt ADF
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE));


        // Listen for new RGB camera data
        if(mTango != null) {
            mCameraView.connectToTangoCamera(mTango, TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
        } else {
            Log.e("PointCloudFragment", "mTango == null");
        }

        mTango.connectListener(framePairs, new Tango.OnTangoUpdateListener() {
            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzij) {
            }

            // Listen to Tango Events
            @Override
            public void onTangoEvent(final TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.onTangoEvent(event);
                }
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        //mTangoEventTextView.setText(event.eventKey + ": " + event.eventValue);
                    }
                });
            }

            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }

                // Make sure to have atomic access to Tango Data so that
                // render loop doesn't interfere while Pose call back is updating
                // the data.
                synchronized (sharedLock) {
                    float[] translation = pose.getTranslationAsFloats();
                    boolean updateRenderer = false;

                    // Check for Device wrt ADF pose, Device wrt Start of Service pose,
                    // Start of Service wrt ADF pose(This pose determines if device
                    // the is relocalized or not).
                    if (pose.baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION
                            && pose.targetFrame == TangoPoseData.COORDINATE_FRAME_DEVICE) {
                        mPoses[0] = pose;
                        if (mAdf2DevicePreviousPoseStatus != pose.statusCode) {
                            // Set the count to zero when status code changes.
                            mAdf2DevicePoseCount = 0;
                        }
                        mAdf2DevicePreviousPoseStatus = pose.statusCode;
                        mAdf2DevicePoseCount++;

                    } else if (pose.baseFrame == TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE
                            && pose.targetFrame == TangoPoseData.COORDINATE_FRAME_DEVICE) {
                        mPoses[1] = pose;
                        if (mStart2DevicePreviousPoseStatus != pose.statusCode) {
                            // Set the count to zero when status code changes.
                            mStart2DevicePoseCount = 0;
                        }
                        mStart2DevicePreviousPoseStatus = pose.statusCode;
                        mStart2DevicePoseCount++;

                    } else if (pose.baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION
                            && pose.targetFrame == TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE) {
                        mPoses[2] = pose;
                        if (mAdf2StartPreviousPoseStatus != pose.statusCode) {
                            // Set the count to zero when status code changes.
                            mAdf2StartPoseCount = 0;
                        }
                        mAdf2StartPreviousPoseStatus = pose.statusCode;
                        mAdf2StartPoseCount++;

                        if (pose.statusCode == TangoPoseData.POSE_VALID && mAdf2StartPoseCount > 1) {
                            mIsRelocalized = true;
                            mRelocalizedToastView.show();
                        } else {
                            mIsRelocalized = false;
                        }
                    }
                }
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // Show color camera as background
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                    mCameraView.onFrameAvailable();
                }
            }
        });
    }

    private void saveAdf() {
        showSetNameDialog();
    }

    private void showSetNameDialog() {
        Bundle bundle = new Bundle();
        if (mCurrentUUID != null) {
            try {
                TangoAreaDescriptionMetaData metaData = mTango
                        .loadAreaDescriptionMetaData(mCurrentUUID);
                byte[] adfNameBytes = metaData.get(TangoAreaDescriptionMetaData.KEY_NAME);
                if (adfNameBytes != null) {
                    String fillDialogName = new String(adfNameBytes);
                    bundle.putString(TangoAreaDescriptionMetaData.KEY_NAME, fillDialogName);
                }
            } catch (TangoErrorException e) {
            }
            bundle.putString(TangoAreaDescriptionMetaData.KEY_UUID, mCurrentUUID);
        }
        FragmentManager manager = getFragmentManager();
        SetADFNameDialog setADFNameDialog = new SetADFNameDialog();
        setADFNameDialog.setArguments(bundle);
        setADFNameDialog.show(manager, "ADFNameDialog");
    }

    @Override
    public void onSetName(String name, String uuids) {

        TangoAreaDescriptionMetaData metadata = new TangoAreaDescriptionMetaData();
        try {
            mCurrentUUID = mTango.saveAreaDescription();
            metadata = mTango.loadAreaDescriptionMetaData(mCurrentUUID);
            metadata.set(TangoAreaDescriptionMetaData.KEY_NAME, name.getBytes());
            mTango.saveAreaDescriptionMetadata(mCurrentUUID, metadata);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), getString(R.string.TangoError),
                    Toast.LENGTH_SHORT).show();
            return;
        } catch (TangoInvalidException e) {
            Toast.makeText(getApplicationContext(), getString(R.string.TangoInvalid),
                    Toast.LENGTH_SHORT).show();
            return;
        }
        Toast.makeText(getApplicationContext(), getString(R.string.adf_save) + mCurrentUUID,
                Toast.LENGTH_SHORT).show();
    }

    /**
     * Updates the text view in UI screen with the Pose. Each pose is associated with Target and
     * Base Frame. We need to check for that pair and update our views accordingly.
     *
     * @param pose
     */
    private void updateTextViews() {
        if (mPoses[0] != null
                && mPoses[0].baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION
                && mPoses[0].targetFrame == TangoPoseData.COORDINATE_FRAME_DEVICE) {
            mAdf2DeviceTranslationTextView.setText(getTranslationString(mPoses[0]));
            mAdf2DeviceQuatTextView.setText(getQuaternionString(mPoses[0]));
            mAdf2DevicePoseStatusTextView.setText(getPoseStatus(mPoses[0]));
            mAdf2DevicePoseCountTextView.setText(Integer.toString(mAdf2DevicePoseCount));
        }

        if (mPoses[1] != null
                && mPoses[1].baseFrame == TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE
                && mPoses[1].targetFrame == TangoPoseData.COORDINATE_FRAME_DEVICE) {
            mStart2DeviceTranslationTextView.setText(getTranslationString(mPoses[1]));
            mStart2DeviceQuatTextView.setText(getQuaternionString(mPoses[1]));
            mStart2DevicePoseStatusTextView.setText(getPoseStatus(mPoses[1]));
            mStart2DevicePoseCountTextView.setText(Integer.toString(mStart2DevicePoseCount));
        }

        if (mPoses[2] != null
                && mPoses[2].baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION
                && mPoses[2].targetFrame == TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE) {
            mAdf2StartTranslationTextView.setText(getTranslationString(mPoses[2]));
            mAdf2StartQuatTextView.setText(getQuaternionString(mPoses[2]));
            mAdf2StartPoseStatusTextView.setText(getPoseStatus(mPoses[2]));
            mAdf2StartPoseCountTextView.setText(Integer.toString(mAdf2StartPoseCount));
        }
    }

    private String getTranslationString(TangoPoseData pose) {
        return "[" + threeDec.format(pose.translation[0]) + ","
                + threeDec.format(pose.translation[1]) + "," + threeDec.format(pose.translation[2])
                + "] ";

    }

    private String getQuaternionString(TangoPoseData pose) {
        return "[" + threeDec.format(pose.rotation[0]) + "," + threeDec.format(pose.rotation[1])
                + "," + threeDec.format(pose.rotation[2]) + "," + threeDec.format(pose.rotation[3])
                + "] ";

    }

    private String getPoseStatus(TangoPoseData pose) {
        switch (pose.statusCode) {
            case TangoPoseData.POSE_INITIALIZING:
                return getString(R.string.pose_initializing);
            case TangoPoseData.POSE_INVALID:
                return getString(R.string.pose_invalid);
            case TangoPoseData.POSE_VALID:
                return getString(R.string.pose_valid);
            default:
                return getString(R.string.pose_unknown);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        mTangoUx.stop();
        try {
            mTango.disconnect();
            mIsTangoServiceConnected = false;
            Log.e("onPause", "disconnect");
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        mTangoUx.start();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            mTango.disconnect();
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT)
                    .show();
        }
    }

    // OnClick Button Listener for all the buttons
    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.saveAdf:
                saveAdf();
                break;
            default:
                Log.w(TAG, "Unknown button click");
                return;
        }
    }

    /**
     * Create a separate thread to update Log information on UI at the specified interval of
     * UPDATE_INTERVAL_MS. This function also makes sure to have access to the mPoses atomically.
     */
    private void startUIThread() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    try {
                        Thread.sleep(UPDATE_INTERVAL_MS);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                try {
                                    synchronized (sharedLock) {

                                        if (mPoses == null) {
                                            return;
                                        } else {
                                            updateTextViews();
                                        }
                                    }
                                } catch (NullPointerException e) {
                                    e.printStackTrace();
                                }
                            }
                        });
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }).start();
    }
}
