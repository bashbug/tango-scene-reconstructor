package bashbug.cloud2mesh;

import android.app.Activity;
import android.content.Intent;
import android.support.v4.app.ActionBarDrawerToggle;
import android.support.v4.view.GravityCompat;
import android.os.Bundle;
import android.app.ActionBar;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.support.v4.widget.DrawerLayout;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.FrameLayout;
import android.widget.ListView;
import android.widget.Toast;

/**
 * Basic Activity to handle a navigation drawer for multiple activities
 * Adds an icon for extra information dialog while area learning and point cloud activity
 */

public class BaseActivity extends Activity {

    protected DrawerLayout drawerLayout;
    protected ListView drawerList;
    protected String[] layers;
    protected FrameLayout content;
    private ActionBarDrawerToggle drawerToggle;
    private boolean mShowDescription = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);

        FrameLayout frameLayout = (FrameLayout) findViewById(R.id.description_textview);
        frameLayout.setVisibility(View.GONE);

        return super.onCreateOptionsMenu(menu);
    }

    public void openDescription() {

        FrameLayout frameLayout = (FrameLayout) findViewById(R.id.description_textview);

        if(mShowDescription) {
            frameLayout.setVisibility(View.GONE);
            mShowDescription = false;
        } else {
            frameLayout.setVisibility(View.VISIBLE);
            mShowDescription = true;
        }

    }

    protected void setupNavDrawer() {

        drawerLayout = (DrawerLayout) findViewById(R.id.drawer_layout);
        layers = getResources().getStringArray(R.array.activities);
        drawerList = (ListView) findViewById(R.id.left_drawer);
        content = (FrameLayout) findViewById(R.id.content_frame);

        //Creating the drawer with items..
        drawerList.setAdapter(new ArrayAdapter<String>(this, R.layout.drawer_list_item, layers));
        drawerList.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> arg0, View arg1, int pos, long arg3) {
            Intent nextActivity = null;

            switch(pos){
                case 0: //Nearby Cabs
                    nextActivity = new Intent(getApplicationContext(), AreaLearningActivity.class);
                    break;
                case 1: //Cab Companies
                    nextActivity = new Intent(getApplicationContext(), PointCloudActivity.class);
                    break;
            }

            if(nextActivity != null) {
                startActivity(nextActivity);
                finish();
            }
            }
        });

        // set the ActionBar app icon to open drawer
        ActionBar actionBar = getActionBar();
        actionBar.setDisplayHomeAsUpEnabled(true);
        actionBar.setHomeButtonEnabled(true);
        drawerToggle = new ActionBarDrawerToggle((Activity) this, drawerLayout, R.drawable.ic_menu, 0, 0) {

            public void onDrawerOpened(View drawerView)
            {
                getActionBar().setTitle(R.string.app_name);
            }
        };

        drawerLayout.setDrawerListener(drawerToggle);

    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        // Handle presses on the action bar items
        if(item.getItemId() == R.id.ic_description) {
            openDescription();
        }

        // Handle item selection
        if(item.getItemId() == android.R.id.home) {
            if (drawerLayout.isDrawerVisible(GravityCompat.START)) {
                drawerLayout.closeDrawer(GravityCompat.START);
            } else {
                drawerLayout.openDrawer(GravityCompat.START);
            }
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void setContentView(int layoutResID) {
        super.setContentView(layoutResID);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == 0) {
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission, Toast.LENGTH_SHORT).show();
                finish();
            }
        } else if (requestCode == 1) {
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.arealearningpermission, Toast.LENGTH_SHORT).show();
                finish();
            }
        }
    }

    @Override
    protected void onPostCreate(Bundle savedInstanceState) {
        super.onPostCreate(savedInstanceState);
        setupNavDrawer();
    }
}