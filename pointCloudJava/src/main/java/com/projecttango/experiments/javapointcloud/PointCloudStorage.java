package com.projecttango.experiments.javapointcloud;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
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

    public void createPCDFile() {
        mPointCloudFile = new File(getDir(), "pcd-" + timeAsString() + ".PCD");
        Log.e("PointCloudStorage", "create File");
        setHeader(mPointCloudFile);
    }

    private void setHeader(File pcdFile) {
        String header;
        header = "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1\nWIDTH 1\nHEIGHT 1\n";

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
        String header = "VIEWPOINT ";
        for (int i=0; i < translation.length; i++) {
            header += String.valueOf(translation[i]) + " ";
        }
        for (int i=0; i < rotation.length; i++) {
            header += String.valueOf(rotation[i]) + " ";
        }
        header += "\nPOINTS " + pointCount +"\nDATA ascii\n";

        try{
            // fulfill header description
            FileWriter fw = new FileWriter(mPointCloudFile, true); //the true will append the new data
            fw.write(header);

            // write point cloud data into pcd file in ascii format
            int k = 0;
            for (int i = 0; i < data.length; i++) {
                if (k == 0 || k == 1) {
                    fw.write(String.valueOf(data[i])+" ");
                    k++;
                }
                if (k == 2) {
                    fw.write(String.valueOf(data[i])+"\n");
                    k=0;
                }
            }

            Log.e("PointCloudStorage", "update File");
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


}
