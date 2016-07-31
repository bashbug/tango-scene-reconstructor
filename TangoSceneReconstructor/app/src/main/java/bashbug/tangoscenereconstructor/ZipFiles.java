package bashbug.tangoscenereconstructor;

import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/**
 * Created by anastasia on 21.04.16.
 */
public class ZipFiles {

    private static final String TAG = "ZipFiles";

    public String setFolder(String dirName) {
        String zipName = dirName + ".zip";
        File dirPath = new File(dirName);
        try {
            FileOutputStream fout = new FileOutputStream(zipName);
            ZipOutputStream zout = new ZipOutputStream(fout);
            Log.e(TAG, "Zip dir: " + dirName + " starts...");
            addSubDir("", dirPath, zout);
            Log.e(TAG, "Zip dir: " + dirName + " stops...");
            zout.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return zipName;
    }

    private static void addSubDir(String basePath, File dir, ZipOutputStream zout) throws IOException {
        byte[] buffer = new byte[4096];
        File[] files = dir.listFiles();
        for (File file : files) {
            if (file.isDirectory()) {
                String path = basePath + file.getName() + "/";
                zout.putNextEntry(new ZipEntry(path));
                addSubDir(path, file, zout);
                zout.closeEntry();
            } else {
                FileInputStream fin = new FileInputStream(file);
                zout.putNextEntry(new ZipEntry(basePath + file.getName()));
                int length;
                while ((length = fin.read(buffer)) > 0) {
                    zout.write(buffer, 0, length);
                }
                zout.closeEntry();
                fin.close();
            }
        }
    }
}
