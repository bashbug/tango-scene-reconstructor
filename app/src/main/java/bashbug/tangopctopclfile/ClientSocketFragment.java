package bashbug.tangopctopclfile;

import android.app.Activity;
import android.net.Uri;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.view.View;
import android.view.View.OnClickListener;
import android.os.AsyncTask;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import android.util.Log;


/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link ClientSocketFragment.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link ClientSocketFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class ClientSocketFragment extends android.support.v4.app.Fragment {
    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String POSITION = "index";

    // TODO: Rename and change types of parameters
    private int mPosition;
    private static final String TAG = SocketCommunication.class.getSimpleName();

    private TextView mTextResponse;
    private EditText mEditTextAddress, mEditTextPort;
    private Button mButtonConnect, mButtonClear, mButtonSend;
    private OnFragmentInteractionListener mListener;

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param mPosition Parameter 1.
     * @return A new instance of fragment ClientSocketFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static ClientSocketFragment newInstance(int position) {
        ClientSocketFragment fragment = new ClientSocketFragment();
        Bundle args = new Bundle();
        args.putInt(POSITION, position);
        fragment.setArguments(args);
        return fragment;
    }

    public ClientSocketFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mPosition = getArguments().getInt(POSITION, 0);
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment

        View view = inflater.inflate(R.layout.activity_socket_communication, container, false);

        mEditTextAddress = (EditText) view.findViewById(R.id.ip_address_input);
        mEditTextPort = (EditText) view.findViewById(R.id.port_input);
        mButtonConnect = (Button) view.findViewById(R.id.connect_button);
        mButtonClear = (Button) view.findViewById(R.id.clear_button);
        mButtonSend = (Button) view.findViewById(R.id.send_button);
        mTextResponse = (TextView) view.findViewById(R.id.response_view);

        mButtonConnect.setOnClickListener(buttonConnectOnClickListener);
        mButtonClear.setOnClickListener(buttonClearOnClickListener);
        mButtonSend.setOnClickListener(buttonSendOnClickListener);

        return view;
    }

    OnClickListener buttonSendOnClickListener = new OnClickListener() {
        @Override
        public void onClick(View v) {
            mTextResponse.setText("");
            mEditTextAddress.setText("");
            mEditTextPort.setText("");
        }
    };

    OnClickListener buttonClearOnClickListener = new OnClickListener() {
        @Override
        public void onClick(View v) {
            mTextResponse.setText("");
            mEditTextAddress.setText("");
            mEditTextPort.setText("");
        }
    };

    OnClickListener buttonConnectOnClickListener = new OnClickListener() {
        @Override
        public void onClick(View v) {
            ClientThread client = new ClientThread(
                    mEditTextAddress.getText().toString(),
                    Integer.parseInt(mEditTextPort.getText().toString()));
            client.execute();
        }
    };

    // TODO: Rename method, update argument and hook method into UI event
    public void onButtonPressed(Uri uri) {
        if (mListener != null) {
            mListener.onFragmentInteraction(uri);
        }
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        try {
            mListener = (OnFragmentInteractionListener) activity;

            ((MainActivity) activity).onSectionAttached(
                    getArguments().getInt(POSITION));

        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement OnFragmentInteractionListener");
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
        mListener = null;
    }

    /**
     * This interface must be implemented by activities that contain this
     * fragment to allow an interaction in this fragment to be communicated
     * to the activity and potentially other fragments contained in that
     * activity.
     * <p/>
     * See the Android Training lesson <a href=
     * "http://developer.android.com/training/basics/fragments/communicating.html"
     * >Communicating with Other Fragments</a> for more information.
     */
    public interface OnFragmentInteractionListener {
        // TODO: Update argument type and name
        public void onFragmentInteraction(Uri uri);

    }

    class ClientThread extends AsyncTask<Void, Void, Void> {

        String dstAddress;
        int dstPort;
        String response = "test";

        ClientThread(String addr, int port){
            dstAddress = addr;
            dstPort = port;
        }

        @Override
        protected Void doInBackground(Void... arg0) {

            Socket socket = null;

            try {
                socket = new Socket(dstAddress, dstPort);

                ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream(1024);
                byte[] buffer = new byte[1024];
                int bytesRead;

                InputStream inputStream = socket.getInputStream();
                Log.e(TAG, "socket establised");

                while ((bytesRead = inputStream.read(buffer)) != -1){
                    Log.e(TAG, "read inputstream");
                    byteArrayOutputStream.write(buffer, 0, bytesRead);
                    response += byteArrayOutputStream.toString("UTF-8");
                    Log.e(TAG, "response:" + response);
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
                if(socket != null){
                    try {
                        socket.close();
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
            mTextResponse.setText(response);
            super.onPostExecute(result);
        }
    }

}
