package bashbug.tangopctopclfile;

import android.os.AsyncTask;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Created by anastasia on 6/26/15.
 */
// task for socket communication
class ClientTask extends AsyncTask<Void, Void, Void> {

    String dstAddress;
    int dstPort;
    String response = null;
    String mFile;

    public ClientTask(String addr, int port, String file) {
        Log.e("ClientTask", "initialized");
        Log.e("ClientTask", "FilePath: " + file);
        Log.e("ClientTask", "ServerAdd: " + addr);
        Log.e("ClientTask", "ServerPort: " + port);
        mFile = file;
        dstAddress = addr;
        dstPort = port;
    }

    private void copy(InputStream in, OutputStream out) throws IOException {
        byte[] buf = new byte[8192];
        int len = 0;
        while ((len = in.read(buf)) != -1) {
            out.write(buf, 0, len);
        }
    }

    @Override
    protected Void doInBackground(Void... arg0) {

        Socket mSocket = null;

        try {
            mSocket = new Socket(dstAddress, dstPort);

            try {
                if (mFile != "##1##") {
                    InputStream is = new FileInputStream(mFile);
                    OutputStream os = mSocket.getOutputStream();
                    copy(is, os);
                    os.close();
                    is.close();
                }
            } catch (IOException e) {
                e.printStackTrace();
                response = "IOException: " + e.toString();
            }

        } catch (UnknownHostException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            response = "UnknownHostException: " + e.toString();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            response = "IOException: " + e.toString();
        } finally {
            if(mSocket != null){
                try {
                    mSocket.close();
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
        return null;
    }

    @Override
    protected void onPostExecute(Void result) {
        //mTextResponse.setText(response);
        super.onPostExecute(result);
        Log.e("ClientSocketFragment", "Client Task is finished");
        if (response == null) {

        }
    }
}