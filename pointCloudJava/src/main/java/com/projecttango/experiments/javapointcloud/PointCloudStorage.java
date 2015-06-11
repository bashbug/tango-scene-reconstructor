package com.projecttango.experiments.javapointcloud;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

/**
 * Created by Anastasia Tondera on 4/20/15.
 * Handles file management to store point cloud data into pcl file.
 */

public class PointCloudStorage {
    File mPointCloudFile;

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
            Log.e("PointCloudStorage", "Directory created");
        }
        return file;
    }

    public void createPCDFile(int count) {
        mPointCloudFile = new File(getDir(), "pcd-" + timeAsString() + "-" + Integer.toString(count) + ".PCD");
        Log.e("PointCloudStorage", "create File");
        setHeader(mPointCloudFile);
    }

    private void setHeader(File pcdFile) {
        String header;
        header = "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n";

        try {
            FileWriter fw = new FileWriter(pcdFile, true);
            fw.write(header);
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String timeAsString() {
        Date date = new Date() ;
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd-hhmmss") ;
        return dateFormat.format(date);
    }

    public void updateFile(byte[] data, int pointCount, float[] translation, float[] rotation) {


        // pcl: translation x,y,z rotation w,x,y,z
        // tango: translation x,y,z rotation x,y,z,w
        String header = "WIDTH " + String.valueOf(pointCount) + "\n" + "HEIGHT 1\nVIEWPOINT ";
        for (int i=0; i < translation.length; i++) {
            header += String.valueOf(translation[i]) + " ";
        }
        for (int i=0; i < rotation.length; i=i+4) {
            // w
            header += String.valueOf(rotation[i+3]) + " ";
            // x
            header += String.valueOf(rotation[i]) + " ";
            // y
            header += String.valueOf(rotation[i+1]) + " ";
            // z
            header += String.valueOf(rotation[i+2]) + " ";
        }
        header += "\nPOINTS " + pointCount +"\nDATA ascii\n";

        try{
            // fulfill header description
            FileWriter fw = new FileWriter(mPointCloudFile, true); //the true will append the new data
            fw.write(header);

            FloatBuffer mPointCloudFloatBuffer;
            mPointCloudFloatBuffer = ByteBuffer.wrap(data) // Wraps a byte array into a buffer
                    .order(ByteOrder.nativeOrder()).asFloatBuffer();

            // write point cloud data into pcd file in ascii format

/*
            for (int i=0; i < mPointCloudFloatBuffer.array().length; i++) {
                fw.write(String.valueOf(mPointCloudFloatBuffer.array()[i]));
            }
*/

            for (int i = 0; i <= mPointCloudFloatBuffer.capacity() - 3; i = i + 3) {
                // x
                fw.write(String.valueOf(mPointCloudFloatBuffer.get(i))+" ");
                // y
                fw.write(String.valueOf(mPointCloudFloatBuffer.get(i+1))+" ");
                // z
                fw.write(String.valueOf(mPointCloudFloatBuffer.get(i+2))+"\n");
            }

            Log.e("PointCloudStorage", "update File");

            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


}
