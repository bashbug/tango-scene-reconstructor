package bashbug.tangopctopclfile;

import android.app.Activity;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.FileObserver;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.view.View.OnClickListener;
import android.os.AsyncTask;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringReader;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;

import android.util.Log;
import android.widget.Toast;


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
    private static final String TAG = ClientSocketFragment.class.getSimpleName();

    private TextView mTextResponse;
    private EditText mEditTextAddress, mEditTextPort;
    private Button mButtonConnect, mButtonClear, mButtonSend;
    private OnFragmentInteractionListener mListener;

    private MainActivity mMainActivity;

    private String mServerSocketAddress;
    private int mServerSocketPort;

    MonitoringAndWritingFileViaSocket mFileObserver;
    File mFileDirection;

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
        Log.e("ClientSocketFragment", "onCreate");
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mPosition = getArguments().getInt(POSITION, 0);
        }

        mFileDirection = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "TangoPointCloud");

        if (!mFileDirection.mkdirs() && !mFileDirection.isDirectory()) {
            Log.e("ClientSocketFragment", "Directory not created");
        } else {
            Log.e("ClientSocketFragment", "Directory created");
        }

        mFileObserver = new MonitoringAndWritingFileViaSocket(mFileDirection.getPath());
        mFileObserver.startWatching();
        mListener.FileObserverCreated(mFileObserver);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        Log.e("ClientSocketFragment", "onCreateView");

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
            mServerSocketAddress = mEditTextAddress.getText().toString();
            mServerSocketPort = Integer.parseInt(mEditTextPort.getText().toString());
            mFileObserver.setServerAddress(mServerSocketAddress);
            mFileObserver.setServerSocketPort(mServerSocketPort);
            ClientTask client = new ClientTask(
                    mEditTextAddress.getText().toString(),
                    Integer.parseInt(mEditTextPort.getText().toString()), "##1##");
            mListener.ClientTaskCreated(client);
            // doInBackground will not fire, because some android versions run tasks parallel
            // and others sequential
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
                client.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
            } else {
                client.execute();
            }
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
        Log.e("ClientSocketFragment", "onAttach");
        super.onAttach(activity);
        Log.e("ClientSocketFragment", "attached");
        try {
            mListener = (OnFragmentInteractionListener) activity;

            ((MainActivity) activity).onSectionAttached(
                    getArguments().getInt(POSITION));

        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement OnFragmentInteractionListener");
        }
/*        Log.e("ClientSocketFragment", "onAttach");
        mFileDirection = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), "TangoPointCloud");

        if (!mFileDirection.mkdirs() && !mFileDirection.isDirectory()) {
            Log.e("ClientSocketFragment", "Directory not created");
        } else {
            Log.e("ClientSocketFragment", "Directory created");
        }

        mFileObserver = new MonitoringAndWritingFileViaSocket(mFileDirection.getPath());
        mFileObserver.startWatching();*/
    }

    @Override
    public void onActivityCreated(Bundle savedInstanceState) {
        Log.e("ClientSocketFragment", "onActivityCreated");
        super.onActivityCreated(savedInstanceState);
    }

    @Override
    public void onDetach() {
        Log.e("ClientSocketFragment", "onDetach");
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

        public void ClientTaskCreated(ClientTask ct);

        public void FileObserverCreated(FileObserver fo);

    }
}
