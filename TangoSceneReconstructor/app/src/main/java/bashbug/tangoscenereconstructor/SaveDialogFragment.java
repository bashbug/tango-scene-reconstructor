package bashbug.tangoscenereconstructor;

import android.app.Activity;
import android.app.DialogFragment;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.TextView;

import bashbug.tangoscenereconstructor.R;

public class SaveDialogFragment extends DialogFragment implements View.OnClickListener {

    private int mSaveMode = 0;
    private boolean mSavePCLMesh = false;
    private boolean mSaveSinglePCDFiles = false;
    private boolean mSaveMergedPCDFiles = false;
    private String mTitle;

    public void setTitle(String title) {
        mTitle = title;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_save_dialog, container, false);
        getDialog().setTitle(mTitle);

        v.findViewById(R.id.Button_Cancel).setOnClickListener(this);
        v.findViewById(R.id.Button_Save).setOnClickListener(this);
        v.findViewById(R.id.pcl_VTK_mesh_checkBox).setOnClickListener(this);
        v.findViewById(R.id.single_PCD_files_check_box).setOnClickListener(this);
        v.findViewById(R.id.merged_PCD_files_check_box).setOnClickListener(this);

        // Watch for button clicks.
        /*Button button = (Button)v.findViewById(R.id.show);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // When button is clicked, call up to owning activity.
                ((FragmentDialog)getActivity()).showDialog();
            }
        });*/

        return v;
    }

    @Override
    public void onClick(View v) {
        Log.e("Dialog", "Click");
        switch (v.getId()) {
            case R.id.Button_Cancel:
                dismiss();
                break;
            case R.id.Button_Save:
                ((MainActivity) getActivity()).SaveFiles(GetSaveMode());
                if(mTitle.equals("Zip&Share reconstruction")) {
                    Log.e("", "ZIP AND SHARE");
                    ((MainActivity) getActivity()).ZipAndShare();
                }
                dismiss();
                break;
            case R.id.pcl_VTK_mesh_checkBox:
                if (((CheckBox) v).isChecked()) {
                    mSavePCLMesh = true;
                } else {
                    mSavePCLMesh = false;
                }
                break;
            case R.id.single_PCD_files_check_box:
                if (((CheckBox) v).isChecked()) {
                    mSaveSinglePCDFiles = true;
                } else {
                    mSaveSinglePCDFiles = false;
                }
                break;
            case R.id.merged_PCD_files_check_box:
                if (((CheckBox) v).isChecked()) {
                    mSaveMergedPCDFiles = true;
                } else {
                    mSaveMergedPCDFiles = false;
                }
                break;
        }
    }

    private int GetSaveMode() {
        // 0 0 0
        if (!mSavePCLMesh && !mSaveSinglePCDFiles && !mSaveMergedPCDFiles) {
            return 0;
        }
        // 0 0 1
        if (!mSavePCLMesh && !mSaveSinglePCDFiles && mSaveMergedPCDFiles) {
            return 1;
        }
        // 0 1 0
        if (!mSavePCLMesh && mSaveSinglePCDFiles && !mSaveMergedPCDFiles) {
            return 2;
        }
        // 0 1 1
        if (!mSavePCLMesh && mSaveSinglePCDFiles && mSaveMergedPCDFiles) {
            return 3;
        }
        // 1 0 0
        if (mSavePCLMesh && !mSaveSinglePCDFiles && !mSaveMergedPCDFiles) {
            return 4;
        }
        // 1 0 1
        if (mSavePCLMesh && !mSaveSinglePCDFiles && mSaveMergedPCDFiles) {
            return 5;
        }
        // 1 1 0
        if (mSavePCLMesh && mSaveSinglePCDFiles && !mSaveMergedPCDFiles) {
            return 6;
        }
        // 1 1 1
        if (mSavePCLMesh && mSaveSinglePCDFiles && mSaveMergedPCDFiles) {
            return 7;
        }
        return 0;
    }

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
    }
}