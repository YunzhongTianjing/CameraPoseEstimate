package com.camera.pose.estimate.test;

import android.graphics.Color;

import com.camera.pose.estimate.algorithm.coplanar_posit.Point;
import com.camera.pose.estimate.algorithm.vpe_old.ImagePoint;
import com.camera.pose.estimate.algorithm.vpe_old.RealPoint;
import com.camera.pose.estimate.algorithm.vpe_old.VolumePoseEst;
import com.camera.pose.estimate.common.DebugView;
import com.camera.pose.estimate.math.Vector3;

public class VPETest {
	public final int WIDTH;
	public final int HEIGHT;

	public static final Vector3[] MODEL = {
			//
			new Vector3(-1, 1, 0),//
			new Vector3(1, 1, 0),//
			new Vector3(1, -1, 0),//
			new Vector3(-1, -1, 0),//
	};
	private final Point[][] FEATURE_POINTS;

	public VPETest(int width, int height, Point[][] featurePoints) {
		this.WIDTH = width;
		this.HEIGHT = height;
		this.FEATURE_POINTS = featurePoints;
	}

	public void check(int index, DebugView debugView) {
		//
		final ImagePoint[] imagePoints = {
				//
				new ImagePoint(FEATURE_POINTS[index][0].x, FEATURE_POINTS[index][0].y),//
				new ImagePoint(FEATURE_POINTS[index][1].x, FEATURE_POINTS[index][1].y),//
				new ImagePoint(FEATURE_POINTS[index][2].x, FEATURE_POINTS[index][2].y),//
				new ImagePoint(FEATURE_POINTS[index][3].x, FEATURE_POINTS[index][3].y),//
		};
		VolumePoseEst vpe = new VolumePoseEst(1, imagePoints, WIDTH, HEIGHT);
		RealPoint[] pointCoordsInCameraSys = vpe.pointCoordsInCameraSys;
		float meanF = vpe.meanF;
		for (int i = 0; i < pointCoordsInCameraSys.length; i++) {
			final RealPoint point = pointCoordsInCameraSys[i];
			final float x = point.x * (meanF / point.z);
			final float y = point.y * (meanF / point.z);
//			final ImagePoint myImagePoint = new ImagePoint(x, y);
			debugView.point(WIDTH / 2 + x, HEIGHT / 2 - y, Color.RED, 10);
		}

		// final Matrix33 rotation = YMath.arrayToMatrix33(vpe.matrixViewModel);
		// final Vector3 translation = YMath.arrayToVector3(vpe.)

		// final Point origin = reproject(rotation, translation, new Vector3(0,
		// 0, 0));
		// final Point xAxis = reproject(rotation, translation, new Vector3(1,
		// 0, 0));
		// final Point yAxis = reproject(rotation, translation, new Vector3(0,
		// 1, 0));
		// final Point zAxis = reproject(rotation, translation, new Vector3(0,
		// 0, -1));
		// debugView.line(origin.x, origin.y, xAxis.x, xAxis.y, Color.RED, 5)
		// .line(origin.x, origin.y, yAxis.x, yAxis.y, Color.GREEN, 5)
		// .line(origin.x, origin.y, zAxis.x, zAxis.y, Color.BLUE, 5);
	}

	// private Point reproject(Matrix33 rotation, Vector3 translation, Vector3
	// point3D) {
	// final Vector3 xxx = Vector3.add(Matrix33.multiply(rotation, point3D),
	// translation);
	// final double imgX = xxx.x * WIDTH / xxx.z;
	// final double imgY = xxx.y * WIDTH / xxx.z;
	// return new Point((float) (WIDTH / 2 + imgX), (float) (HEIGHT / 2 -
	// imgY));
	// }
}
