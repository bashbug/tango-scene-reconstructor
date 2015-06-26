package bashbug.tangopctopclfile;

import android.os.AsyncTask;
import android.os.Build;
import android.os.FileObserver;
import android.util.Log;

/**
 * Created by anastasia on 6/26/15.
 */
// monitoring for new files and write them through socket connection
class MonitoringAndWritingFileViaSocket extends FileObserver {
    String mFileDirection;
    String mServerSocketAddress = null;
    int mServerSocketPort = -1;

    public MonitoringAndWritingFileViaSocket(String path) {
        super(path);
        mFileDirection = path;
        Log.e("FileObserver", "initialized");
    }

    public void setServerAddress(String serveradd) {
        mServerSocketAddress = serveradd;
    }

    public void setServerSocketPort(int serverport) {
        mServerSocketPort = serverport;
    }

    @Override
    public void onEvent(int event, String path) {
        Log.e("FileObserver", String.valueOf(event));
        if (event == FileObserver.CLOSE_WRITE) {
            Log.e("FileObserver" , "new file");
            if(mServerSocketAddress != null && mServerSocketPort != -1) {
                Log.e("FileObserver" , "start writing file server...");
                ClientTask client = new ClientTask(mServerSocketAddress, mServerSocketPort, mFileDirection + "/" +path);
                // doInBackground will not fire, because some android versions run tasks parallel
                // and others sequential
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
                    client.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                } else {
                    client.execute();
                }
                Log.e("FileObserver" , "stop writing file server");
            }
        }
    }
}