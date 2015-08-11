package bashbug.cloud2mesh;

import android.opengl.Matrix;
import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

/**
 * Asynchronous task to write point clouds to .PCD files
 */

class SavePointCloudTask extends AsyncTask<Void, Void, Boolean> {

    ArrayList<TangoCoordinateFramePair> mFramePairs;
    TangoXyzIjData mXyzIj;
    int mPclFileCounter;
    float[] mQuaternion;
    TangoPoseData mPoseData;

    File mPointCloudFile_ascii;

    TangoCameraIntrinsics mIntrinsics;

    private static final float[] inVec = new float[4];
    private static final float[] outVec = new float[4];
    private float[] mCam2dev_Transform = new float[16];


    public SavePointCloudTask(float[] cam2dev_Transform, TangoPoseData poseData, TangoXyzIjData xyzIj, int pclFileCounter) {
        Log.e("SendCommandTask", "created");
        mXyzIj = xyzIj;
        mPclFileCounter = pclFileCounter;
        mPoseData = poseData;
        mCam2dev_Transform = cam2dev_Transform;
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

        mPointCloudFile_ascii = new File(getDir(), "pcd-" + timeAsString() + "-" + Integer.toString(mPclFileCounter) + "-ascii.PCD");

        Log.e("PointCloudStorage", "start writing file local");
        // pcl: translation x,y,z rotation w,x,y,z
        // tango: translation x,y,z rotation x,y,z,w
        String header = "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n" +
                "WIDTH " + mXyzIj.xyzCount + "\n" + "HEIGHT 1\nVIEWPOINT ";

        /*header += String.valueOf(mQuaternion[0]) + " " + //tx
                String.valueOf(mQuaternion[1]) + " " + //ty
                String.valueOf(mQuaternion[2]) + " " + //tz
                String.valueOf(mQuaternion[3]) + " " + //rw
                String.valueOf(mQuaternion[4]) + " " + //rx
                String.valueOf(mQuaternion[5]) + " " + //ry
                String.valueOf(mQuaternion[6]); //rz*/

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
        header_ascii += "\nPOINTS " + mXyzIj.xyzCount +"\nDATA ascii\n";



        try{

            // write point cloud data into pcd file in ascii format
            FileWriter fw = new FileWriter(mPointCloudFile_ascii, true); //the true will append the new data
            fw.write(header_ascii);

            for (int i = 0; i <= mXyzIj.xyz.capacity() - 3; i = i + 3) {
                float[] point2Cam = new float[3];
                point2Cam[0] = mXyzIj.xyz.get(i);
                point2Cam[1] = mXyzIj.xyz.get(i+1);
                point2Cam[2] = mXyzIj.xyz.get(i+2);

                float[] point2Dev = transformPointsToDeviceFrame(point2Cam);
                // x = x
                fw.write(String.valueOf(point2Dev[0])+" ");
                // y = -y
                fw.write(String.valueOf(point2Dev[1])+" ");
                // z = -z
                fw.write(String.valueOf(point2Dev[2])+"\n");
            }

            fw.close();

            Log.e("PointCloudStorage", "stop writing file local");

        } catch (IOException e) {
            e.printStackTrace();
        }

        return true;
    }

    @Override
    protected void onPostExecute(Boolean done) {

    }
}