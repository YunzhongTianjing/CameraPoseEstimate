package com.camera.pose.estimate.math;

public class YMath {
	private YMath() {
	}

	public static float[] getYawPitchRoll(double[][] matrix) {
		final Matrix33 matrix33 = new Matrix33();
		matrix33.v00 = (float) matrix[0][0];
		matrix33.v01 = (float) matrix[0][1];
		matrix33.v02 = (float) matrix[0][2];

		matrix33.v10 = (float) matrix[1][0];
		matrix33.v11 = (float) matrix[1][1];
		matrix33.v12 = (float) matrix[1][2];

		matrix33.v20 = (float) matrix[2][0];
		matrix33.v21 = (float) matrix[2][1];
		matrix33.v22 = (float) matrix[2][2];

		final float[] resultYawPitchRoll = new float[3];
		matrix33.getYawPitchRoll(resultYawPitchRoll);
		return resultYawPitchRoll;
	}

	public static Matrix33 arrayToMatrix33(double[][] matrix) {
		final Matrix33 matrix33 = new Matrix33();
		matrix33.v00 = (float) matrix[0][0];
		matrix33.v01 = (float) matrix[0][1];
		matrix33.v02 = (float) matrix[0][2];

		matrix33.v10 = (float) matrix[1][0];
		matrix33.v11 = (float) matrix[1][1];
		matrix33.v12 = (float) matrix[1][2];

		matrix33.v20 = (float) matrix[2][0];
		matrix33.v21 = (float) matrix[2][1];
		matrix33.v22 = (float) matrix[2][2];
		return matrix33;
	}

	public static Matrix33 arrayToMatrix33(float[][] matrix) {
		final Matrix33 matrix33 = new Matrix33();
		matrix33.v00 = matrix[0][0];
		matrix33.v01 = matrix[0][1];
		matrix33.v02 = matrix[0][2];

		matrix33.v10 = matrix[1][0];
		matrix33.v11 = matrix[1][1];
		matrix33.v12 = matrix[1][2];

		matrix33.v20 = matrix[2][0];
		matrix33.v21 = matrix[2][1];
		matrix33.v22 = matrix[2][2];
		return matrix33;
	}

	public static Vector3 arrayToVector3(double[] vector) {
		return new Vector3((float) vector[0], (float) vector[1], (float) vector[2]);
	}

}
