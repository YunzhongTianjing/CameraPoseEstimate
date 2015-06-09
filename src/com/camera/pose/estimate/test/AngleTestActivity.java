package com.camera.pose.estimate.test;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.BitmapFactory.Options;
import android.graphics.Color;
import android.os.Bundle;
import android.view.View;

import com.camera.pose.estimate.R;
import com.camera.pose.estimate.common.DebugView;

public class AngleTestActivity extends Activity {

	private DebugView mDebugView;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		getActionBar().hide();
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_angle_test);
		mDebugView = (DebugView) findViewById(R.id.debugView);
		mDebugView.point(640, 0, Color.BLUE, 10).point(640, 480, Color.BLACK, 10);

		changeImage(null);
	}

	private int clickCounts;

	private POSITTest positTest = new POSITTest(AngleTestData.WIDTH, AngleTestData.HEIGHT, AngleTestData.FEATURE_POINTS);

	public void changeImage(View view) {
		final Bitmap bmp = getBitmapWithNoScale(AngleTestData.IMG_IDS[clickCounts % AngleTestData.TEST_SIZE]);
		positTest.check(clickCounts % AngleTestData.TEST_SIZE, mDebugView);
		clickCounts++;
		mDebugView.setBitmap(bmp).showDebug();
	}

	public Bitmap getBitmapWithNoScale(int resId) {
		final Options options = new Options();
		options.inScaled = false;
		return BitmapFactory.decodeResource(getResources(), resId, options);
	}
}
