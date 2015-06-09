package com.camera.pose.estimate.test;

import java.io.Serializable;

import android.app.Activity;
import android.content.Intent;
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

		initialize(getIntent());

		mDebugView = (DebugView) findViewById(R.id.debugView);
		mDebugView.point(640, 0, Color.BLUE, 10).point(640, 480, Color.BLACK, 10);

		changeImage(null);
	}

	private void initialize(Intent intent) {
		final Algorithm algorithm = (Algorithm) intent.getSerializableExtra(KEY_ALGORITHM);
		final TestData testData = (TestData) intent.getSerializableExtra(KEY_TESTDATA);
		if (testData == TestData.ANGLE) {
			if (algorithm == Algorithm.RPP) {
				test = new RPPTest(AngleTestData.WIDTH, AngleTestData.HEIGHT, AngleTestData.FEATURE_POINTS);
			} else if (algorithm == Algorithm.POSIT) {
				test = new POSITTest(AngleTestData.WIDTH, AngleTestData.HEIGHT, AngleTestData.FEATURE_POINTS);
			}
		}
	}

	private int clickCounts;

	private ATest test;

	public void changeImage(View view) {
		final Bitmap bmp = getBitmapWithNoScale(AngleTestData.IMG_IDS[clickCounts % AngleTestData.TEST_SIZE]);
		test.checkCoordinate(clickCounts % AngleTestData.TEST_SIZE, mDebugView);
		clickCounts++;
		mDebugView.setBitmap(bmp).showDebug();
	}

	public Bitmap getBitmapWithNoScale(int resId) {
		final Options options = new Options();
		options.inScaled = false;
		return BitmapFactory.decodeResource(getResources(), resId, options);
	}

	public static class Starter {
		private Algorithm algorithm;
		private TestData testData;

		private Starter() {
		}

		public static Starter create() {
			return new Starter();
		}

		public Starter test(Algorithm algorithm) {
			this.algorithm = algorithm;
			return this;
		}

		public Starter with(TestData testData) {
			this.testData = testData;
			return this;
		}

		public void start(Activity source) {
			final Intent intent = new Intent(source, AngleTestActivity.class);
			intent.putExtra(KEY_ALGORITHM, algorithm);
			intent.putExtra(KEY_TESTDATA, testData);
			source.startActivity(intent);
		}
	}

	private static final String KEY_ALGORITHM = "algorithm";
	private static final String KEY_TESTDATA = "testdata";

	public static enum Algorithm implements Serializable {
		RPP, POSIT
	}

	public static enum TestData implements Serializable {
		ANGLE
	}
}
