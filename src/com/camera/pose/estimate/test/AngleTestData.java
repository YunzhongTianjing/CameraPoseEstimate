package com.camera.pose.estimate.test;

import com.camera.pose.estimate.R;
import com.camera.pose.estimate.algorithm.coplanar_posit.Point;

public class AngleTestData {
	public static final int WIDTH = 640;
	public static final int HEIGHT = 480;
	
	public static final int[] IMG_IDS = { R.drawable.angle30,
			R.drawable.angle45, R.drawable.angle60, R.drawable.angle5015,
			R.drawable.angle6030 };
	public static final int TEST_SIZE = IMG_IDS.length;

	public static final Point[][] FEATURE_POINTS = { {
			// angle30
			new Point(250, 183),//
			new Point(387, 184),//
			new Point(400, 313),//
			new Point(241, 313),//
	}, {
			// angle45
			new Point(255, 193),//
			new Point(389, 193),//
			new Point(402, 297),//
			new Point(238, 298),//
	}, {
			// angle60
			new Point(254, 206),//
			new Point(386, 206),//
			new Point(405, 284),//
			new Point(236, 282),//
	}, {
			// angle5015
			new Point(272, 144),//
			new Point(400, 164),//
			new Point(379, 264),//
			new Point(220, 231),//
	}, {
			// angle6030
			new Point(291, 149),//
			new Point(410, 180),//
			new Point(349, 256),//
			new Point(210, 207),//
	}, };
}
