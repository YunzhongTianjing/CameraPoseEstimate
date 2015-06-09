package com.camera.pose.estimate;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;

import com.camera.pose.estimate.test.AngleTestActivity.Algorithm;
import com.camera.pose.estimate.test.AngleTestActivity.Starter;
import com.camera.pose.estimate.test.AngleTestActivity.TestData;

public class MainActivity extends Activity {

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
	}

	public void coplanarPOSIT_AngleTest(View view) {
		Starter.create().test(Algorithm.POSIT).with(TestData.ANGLE).start(this);
	}

	public void rpp_AngleTest(View view) {
		Starter.create().test(Algorithm.RPP).with(TestData.ANGLE).start(this);
	}

}
