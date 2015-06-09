package com.camera.pose.estimate.algorithm.rpp;

import java.text.DecimalFormat;

import com.camera.pose.estimate.math.YMath;

public class RPP {
	static {
		System.loadLibrary("rpp");
	}

	public static class PoseResult {
		public double error;
		public double[] translation = new double[3];
		public double[][] rotation = new double[3][3];

		private static final DecimalFormat FORMAT = new DecimalFormat("#.##");

		@Override
		public String toString() {
			String result = "";
			result += "error is " + FORMAT.format(error) + "\n";
			result += "translation is x:" + FORMAT.format(translation[0]) + ",y:" + FORMAT.format(translation[1]) + ",z:"
					+ FORMAT.format(translation[2]) + "\n";
			final float[] yawPitchRoll = YMath.getYawPitchRoll(rotation);
			result += "rotation is x:" + FORMAT.format(yawPitchRoll[1]) + ",y:" + FORMAT.format(yawPitchRoll[0]) + ",z:"
					+ FORMAT.format(yawPitchRoll[2]) + "\n";
			// result += "translation is " + Arrays.toString(translation) +
			// "\n";
			// result += "rotation is\n";
			// for (int i = 0; i < rotation.length; i++)
			// result += Arrays.toString(rotation[i]) + "\n";
			return result;
		}
	}

	public native static void estimate(double[] principalPoint, double[] focalLength, double[] modelPoints, double[] imagePoints,
			int pointsNum, double[][] R_init, boolean estimateByR_init, double epsilon, double tolerance, int maxIiterations,
			PoseResult poseResult);
}
