package bashbug.rgbpointcloudbuilder;

import android.app.Activity;
import android.app.DialogFragment;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;

/**
 * This Class shows a dialog to set the hostname and ip of a socket server.
 */
public class SetServerSocketAddressAndIPDialog extends DialogFragment implements View.OnClickListener {
    private EditText mEditTextAddress, mEditTextPort;

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
    }

    @Override
    public View onCreateView(LayoutInflater inflator, ViewGroup container, Bundle savedInstanceState) {
        View dialogView = inflator.inflate(R.layout.set_server_socket_address_and_ip_dialog, null);
        getDialog().setTitle(R.string.set_socket_dialogTitle);
        dialogView.findViewById(R.id.Button_Connect).setOnClickListener(this);
        dialogView.findViewById(R.id.Button_Cancel).setOnClickListener(this);
        mEditTextAddress = (EditText) dialogView.findViewById(R.id.destAddress);
        mEditTextPort = (EditText) dialogView.findViewById(R.id.destPort);
        return dialogView;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.Button_Connect:
                connect();
                dismiss();
                break;
            case R.id.Button_Cancel:
                dismiss();
                break;
        }
    }


    public void connect() {
        ((MainActivity)getActivity()).setmSendPCDSwitch(true);
        JNIInterface.setSocket(mEditTextAddress.getText().toString(), Integer.parseInt(mEditTextPort.getText().toString()));
    }
}
