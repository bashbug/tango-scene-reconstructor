package com.projecttango.experiments.javapointcloud;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
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

    public void createFile() {
        Date date = new Date() ;
        SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd-hhmmss") ;
        mPointCloudFile = new File(getDir(), "pcl-" + dateFormat.format(date));
        Log.e("PointCloudStorage", "create File");
    }

    public void updateFile(byte[] data) {
        try{
            FileWriter fw = new FileWriter(mPointCloudFile, true); //the true will append the new data
            fw.write(Arrays.toString(data)); //appends the string to the file
            Log.e("PointCloudStorage", "update File");
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


}
