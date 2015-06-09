package com.camera.pose.estimate.test;

import com.camera.pose.estimate.algorithm.coplanar_posit.Point;
import com.camera.pose.estimate.common.DebugView;
import com.camera.pose.estimate.math.Vector3;

public abstract class ATest {
	public final int WIDTH;
	public final int HEIGHT;

	public static final Vector3[] MODEL = {
			//
			new Vector3(-1, 1, 0),//
			new Vector3(1, 1, 0),//
			new Vector3(1, -1, 0),//
			new Vector3(-1, -1, 0),//
	};
	protected final Point[][] FEATURE_POINTS;

	public ATest(int width, int height, Point[][] featurePoints) {
		this.WIDTH = width;
		this.HEIGHT = height;
		this.FEATURE_POINTS = featurePoints;
	}

	public abstract void checkCoordinate(int index, DebugView debugView);

	public abstract void checkReprojectPoints(int index, DebugView debugView);
}
