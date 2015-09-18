package bashbug.cloud2mesh;

import android.opengl.Matrix;
import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.TimeUnit;

/**
 * Asynchronous task to write point clouds to .PCD files
 */

class SavePointCloudTask extends AsyncTask<Void, Void, Boolean> {

    ArrayList<TangoCoordinateFramePair> mFramePairs;
    float[] mXyzIj;
    int mPclFileCounter;
    float[] mQuaternion;
    TangoPoseData mPoseData;

    BufferedWriter mPointCloudFile_ascii;

    TangoCameraIntrinsics mIntrinsics;

    private static final float[] inVec = new float[4];
    private static final float[] outVec = new float[4];
    private float[] mCam2dev_Transform = new float[16];


    public SavePointCloudTask(TangoPoseData poseData, float[] xyzIj, int pclFileCounter) {
        Log.e("SendCommandTask", "created");
        mXyzIj = xyzIj;
        mPclFileCounter = pclFileCounter;
        mPoseData = poseData;
    }


    public float[] transformPointsToDeviceFrame(float[] point)
    {
        inVec[0] = point[0];
        inVec[1] = point[1];
        inVec[2] = point[2];
        inVec[3] = 1;

        Matrix.multiplyMV(outVec, 0, mCam2dev_Transform, 0, inVec, 0);

        return outVec;
    }


    /* Checks if external storage is available for read and write */
    private boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        return Environment.MEDIA_MOUNTED.equals(state);
    }

    /* Checks if external storage is available to at least read */
    private boolean isExternalStorageReadable() {
        String state = Environment.getExternalStorageState();
        return Environment.MEDIA_MOUNTED.equals(state) ||
                Environment.MEDIA_MOUNTED_READ_ONLY.equals(state);
    }

    private File getDir() {
        if(!isExternalStorageReadable()) {
            Log.e("PointCloudStorage", "External storage not readable");
        }

        if(!isExternalStorageWritable()) {
            Log.e("PointCloudStorage", "External storage not writable");
        }

        // Get the directory for the user's public documents directory.
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "TangoPointCloud");

        if (!file.mkdirs() && !file.isDirectory()) {
            Log.e("PointCloudStorage", "Directory not created");
        } else {
            //Log.e("PointCloudStorage", "Directory created");
        }
        return file;
    }

    public String timeAsString() {
        Date date = new Date() ;
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd-hhmmss") ;
        return dateFormat.format(date);
    }

    @Override
    protected Boolean doInBackground(Void... params) {

        String filename = "pcd-" + timeAsString() + "-" + Integer.toString(mPclFileCounter) + "-ascii.PCD";
        try {
            mPointCloudFile_ascii = new BufferedWriter(new FileWriter(new File(getDir(), filename)));
        } catch (IOException e) {
            e.printStackTrace();
        }

        Date startDate = new Date();
        Log.e("PointCloudStorage", "start writing file '"+filename+"' local");
        // pcl: translation x,y,z rotation w,x,y,z
        // tango: translation x,y,z rotation x,y,z,w
        String header = "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n" +
                "WIDTH " + mXyzIj.length/3 + "\n" + "HEIGHT 1\nVIEWPOINT ";

        float[] translation = mPoseData.getTranslationAsFloats();
        float[] rotation = mPoseData.getRotationAsFloats();

        header += String.valueOf(translation[0]) + " " + //tx
                String.valueOf(translation[1]) + " " + //ty
                String.valueOf(translation[2]) + " " + //tz
                String.valueOf(rotation[3]) + " " + //rw
                String.valueOf(rotation[0]) + " " + //rx
                String.valueOf(rotation[1]) + " " + //ry
                String.valueOf(rotation[2]); //rz

        String header_ascii = header;
        header_ascii += "\nPOINTS " + mXyzIj.length/3 +"\nDATA ascii\n";



        try{

            // write point cloud data into pcd file in ascii format
            //FileWriter fw = new FileWriter(mPointCloudFile_ascii, true); //the true will append the new data
            mPointCloudFile_ascii.write(header_ascii);

            for (int i = 0; i <= mXyzIj.length - 3; i = i + 3) {
                // x = x
                mPointCloudFile_ascii.write(String.valueOf(mXyzIj[i]) + " ");
                // y = -y
                mPointCloudFile_ascii.write(String.valueOf(mXyzIj[i + 1] * -1) + " ");
                // z = -z
                mPointCloudFile_ascii.write(String.valueOf(mXyzIj[i + 2] * -1) + "\n");
            }

            mPointCloudFile_ascii.close();

            Date stopDate = new Date();
            long difference = stopDate.getTime() - startDate.getTime();
            long seconds = TimeUnit.MILLISECONDS.toSeconds(difference);
            Log.e("PointCloudStorage", "stop writing file '"+filename+"' local ("+ difference+"ms)");

        } catch (IOException e) {
            e.printStackTrace();
        }

        return true;
    }

    @Override
    protected void onPostExecute(Boolean done) {

    }
}