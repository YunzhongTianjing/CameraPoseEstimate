package com.camera.pose.estimate.algorithm.vpe_old;

public class MatrixUtils {

	public static void printMatrix(float[][] matrix) {
		for (float[] tempi : matrix) {
			for (float tempj : tempi) {
				System.out.print(tempj + "  ");
			}
			System.out.println();
		}
	}

	public static float[] flatten(float[][] d2array) {
		final float[] result = new float[d2array.length * d2array[0].length];
		for (int i = 0; i < d2array.length; i++) {
			for (int j = 0; j < d2array[0].length; j++)
				result[i + 4 * j] = d2array[i][j];
		}
		return result;
	}
	
	public static float[] flatten(double[][] d2array) {
		final float[] result = new float[d2array.length * d2array[0].length];
		for (int i = 0; i < d2array.length; i++) {
			for (int j = 0; j < d2array[0].length; j++)
				result[i + 4 * j] = (float) d2array[i][j];
		}
		return result;
	}
}