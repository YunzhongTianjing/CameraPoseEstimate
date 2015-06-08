package com.camera.pose.estimate;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

import com.camera.pose.estimate.test.AngleTestActivity;

public class MainActivity extends Activity {

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
	}

	public void coplanarPOSIT_AngleTest(View view) {
		Intent intent = new Intent(this, AngleTestActivity.class);
		startActivity(intent);
	}
}
