package com.camera.pose.estimate.test;

import android.graphics.Color;

import com.camera.pose.estimate.algorithm.coplanar_posit.PlanarPOSIT;
import com.camera.pose.estimate.algorithm.coplanar_posit.Point;
import com.camera.pose.estimate.common.DebugView;
import com.camera.pose.estimate.math.Matrix33;
import com.camera.pose.estimate.math.Vector3;

public class POSITTest {
	public static final int WIDTH = 640;
	public static final int HEIGHT = 480;

	public static final Vector3[] MODEL = {
			//
			new Vector3(-1, 1, 0),//
			new Vector3(1, 1, 0),//
			new Vector3(1, -1, 0),//
			new Vector3(-1, -1, 0),//
	};
	private final PlanarPOSIT POSIT = new PlanarPOSIT(MODEL, WIDTH);

	public void estimate(Point[] featurePoints, DebugView debugView) {
		POSIT.estimatePose(featurePoints, WIDTH, HEIGHT);
		final Matrix33 rotation = POSIT.getBestEstimatedRotation();
		final Vector3 translation = POSIT.getBestEstimatedTranslation();

		final Point origin = reproject(rotation, translation, new Vector3(0, 0, 0));
		final Point xAxis = reproject(rotation, translation, new Vector3(1, 0, 0));
		final Point yAxis = reproject(rotation, translation, new Vector3(0, 1, 0));
		final Point zAxis = reproject(rotation, translation, new Vector3(0, 0, 1));
		debugView.line(origin.x, origin.y, xAxis.x, xAxis.y, Color.RED, 5)
				.line(origin.x, origin.y, yAxis.x, yAxis.y, Color.GREEN, 5)
				.line(origin.x, origin.y, zAxis.x, zAxis.y, Color.BLUE, 5);
	}

	public Point reproject(Matrix33 rotation, Vector3 translation, Vector3 point3D) {
		final Vector3 xxx = Vector3.add(Matrix33.multiply(rotation, point3D), translation);
		final double imgX = xxx.x * WIDTH / xxx.z;
		final double imgY = xxx.y * WIDTH / xxx.z;
		return new Point((float) (WIDTH / 2 + imgX), (float) (HEIGHT / 2 - imgY));
	}
}
