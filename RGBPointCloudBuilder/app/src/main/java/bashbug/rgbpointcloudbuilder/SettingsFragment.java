package bashbug.rgbpointcloudbuilder;

import android.content.Context;
import android.content.SharedPreferences;
import android.net.Uri;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;


/**
 * A simple {@link Fragment} subclass.
 * Activities that contain this fragment must implement the
 * {@link SettingsFragment.OnFragmentInteractionListener} interface
 * to handle interaction events.
 * Use the {@link SettingsFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class SettingsFragment extends PreferenceFragment implements SharedPreferences.OnSharedPreferenceChangeListener {

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Load the preferences from an XML resource
        addPreferencesFromResource(R.xml.preferences);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle
            savedInstanceState) {
        View view = super.onCreateView(inflater, container, savedInstanceState);
        view.setBackgroundColor(getResources().getColor(android.R.color.black));
        return view;
    }

    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        if (key.equals("background_color")) {
            boolean value = sharedPreferences.getBoolean(key, true);
            JNIInterface.setBackgroundColorBlack(value);
        }
        if (key.equals("grid")) {
            boolean value = sharedPreferences.getBoolean(key, true);
            JNIInterface.setGridOn(value);
        }
        if (key.equals("ftfsm") || key.equals("mfsm")) {

            boolean value =  sharedPreferences.getBoolean("ftfsm", true);
            boolean value2 = sharedPreferences.getBoolean("mfsm", true);

            if (value && !value2) {
                JNIInterface.setOptimizationMethods(0);
            } else if (!value && value2) {
                JNIInterface.setOptimizationMethods(1);
            } else {
                JNIInterface.setOptimizationMethods(2);
            }
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        getPreferenceManager().getSharedPreferences().registerOnSharedPreferenceChangeListener(this);

    }

    @Override
    public void onPause() {
        getPreferenceManager().getSharedPreferences().unregisterOnSharedPreferenceChangeListener(this);
        super.onPause();
    }
}
