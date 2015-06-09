package com.camera.pose.estimate.test;

import android.graphics.Color;

import com.camera.pose.estimate.algorithm.coplanar_posit.Point;
import com.camera.pose.estimate.algorithm.rpp.RPP;
import com.camera.pose.estimate.algorithm.rpp.RPP.PoseResult;
import com.camera.pose.estimate.common.DebugView;
import com.camera.pose.estimate.math.Matrix33;
import com.camera.pose.estimate.math.Vector3;
import com.camera.pose.estimate.math.YMath;

public class RPPTest extends ATest {

	private final double[] MODEL_ARRAY;

	public RPPTest(int width, int height, Point[][] featurePoints) {
		super(width, height, featurePoints);
		MODEL_ARRAY = new double[] {
				//
				MODEL[0].x, MODEL[0].y, MODEL[0].z,//
				MODEL[1].x, MODEL[1].y, MODEL[1].z,//
				MODEL[2].x, MODEL[2].y, MODEL[2].z,//
				MODEL[3].x, MODEL[3].y, MODEL[3].z,//
		};
	}

	public void checkCoordinate(int index, DebugView debugView) {
		final double[] principalPoint = { WIDTH / 2, HEIGHT / 2 };
		final double[] focalLength = { WIDTH, WIDTH };
		final double[] imagePoints = {
				//
				FEATURE_POINTS[index][0].x, FEATURE_POINTS[index][0].y, 1,//
				FEATURE_POINTS[index][1].x, FEATURE_POINTS[index][1].y, 1,//
				FEATURE_POINTS[index][2].x, FEATURE_POINTS[index][2].y, 1,//
				FEATURE_POINTS[index][3].x, FEATURE_POINTS[index][3].y, 1,//
		};
		final int pointsNum = 4;
		final double[][] R_init = null;
		final boolean estimateByR_init = false;
		final double epsilon = 0, tolerance = 0;
		final int maxIterations = 0;
		final PoseResult poseResult = new PoseResult();
		RPP.estimate(principalPoint, focalLength, MODEL_ARRAY, imagePoints, pointsNum, R_init, estimateByR_init, epsilon,
				tolerance, maxIterations, poseResult);

		final Matrix33 rotation = YMath.arrayToMatrix33(poseResult.rotation);
		final Vector3 translation = YMath.arrayToVector3(poseResult.translation);

		final Point origin = reproject(rotation, translation, new Vector3(0, 0, 0));
		final Point xAxis = reproject(rotation, translation, new Vector3(1, 0, 0));
		final Point yAxis = reproject(rotation, translation, new Vector3(0, 1, 0));
		final Point zAxis = reproject(rotation, translation, new Vector3(0, 0, 1));
		debugView
				//
				.line(origin.x, origin.y, xAxis.x, xAxis.y, Color.RED, 5)
				.line(origin.x, origin.y, yAxis.x, yAxis.y, Color.GREEN, 5)
				.line(origin.x, origin.y, zAxis.x, zAxis.y, Color.BLUE, 5);
		debugView.text(poseResult.toString());
	}

	public void checkReprojectPoints(int index, DebugView debugView) {
		final double[] principalPoint = { WIDTH / 2, HEIGHT / 2 };
		final double[] focalLength = { WIDTH, WIDTH };
		final double[] imagePoints = {
				//
				FEATURE_POINTS[index][0].x, FEATURE_POINTS[index][0].y, 1,//
				FEATURE_POINTS[index][1].x, FEATURE_POINTS[index][1].y, 1,//
				FEATURE_POINTS[index][2].x, FEATURE_POINTS[index][2].y, 1,//
				FEATURE_POINTS[index][3].x, FEATURE_POINTS[index][3].y, 1,//
		};
		final int pointsNum = 4;
		final double[][] R_init = null;
		final boolean estimateByR_init = false;
		final double epsilon = 0, tolerance = 0;
		final int maxIterations = 0;
		final PoseResult poseResult = new PoseResult();
		RPP.estimate(principalPoint, focalLength, MODEL_ARRAY, imagePoints, pointsNum, R_init, estimateByR_init, epsilon,
				tolerance, maxIterations, poseResult);

		final Matrix33 rotation = YMath.arrayToMatrix33(poseResult.rotation);
		final Vector3 translation = YMath.arrayToVector3(poseResult.translation);

		final Point origin = reproject(rotation, translation, new Vector3((float) MODEL_ARRAY[0], (float) MODEL_ARRAY[1],
				(float) MODEL_ARRAY[2]));
		debugView.point(origin.x, origin.y, Color.RED, 10);
	}

	private Point reproject(Matrix33 rotation, Vector3 translation, Vector3 point3D) {
		final Vector3 xxx = Vector3.add(Matrix33.multiply(rotation, point3D), translation);
		final double imgX = xxx.x * WIDTH / xxx.z;
		final double imgY = xxx.y * WIDTH / xxx.z;
		return new Point((float) (WIDTH / 2 + imgX), (float) (HEIGHT / 2 + imgY));
	}
}
