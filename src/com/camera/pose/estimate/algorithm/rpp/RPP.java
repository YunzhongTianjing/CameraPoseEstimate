package com.camera.pose.estimate.algorithm.rpp;

import java.util.Arrays;

public class RPP {
	static {
		System.loadLibrary("rpp");
	}

	public static class PoseResult {
		public double error;
		public double[] translation = new double[3];
		public double[][] rotation = new double[3][3];

		@Override
		public String toString() {
			String result = "";
			result += "error is " + error + "\n";
			result += "translation is " + Arrays.toString(translation) + "\n";
			result += "rotation is\n";
			for (int i = 0; i < rotation.length; i++)
				result += Arrays.toString(rotation[i]) + "\n";
			return result;
		}
	}

	public native static void estimate(double[] principalPoint,
			double[] focalLength, double[] modelPoints, double[] imagePoints,
			int pointsNum, double[][] R_init, boolean estimateByR_init,
			double epsilon, double tolerance, int maxIiterations,
			PoseResult poseResult);
}
