package com.camera.pose.estimate;

import android.app.Activity;
import android.graphics.Color;
import android.os.Bundle;

import com.camera.pose.estimate.common.DebugView;

public class MainActivity extends Activity {

	private DebugView mDebugView;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		mDebugView = (DebugView) findViewById(R.id.debugView);
		mDebugView.point(100, 100, Color.GREEN, 10)
				.point(200, 200, Color.BLUE, 20).showDebug();
	}
}
