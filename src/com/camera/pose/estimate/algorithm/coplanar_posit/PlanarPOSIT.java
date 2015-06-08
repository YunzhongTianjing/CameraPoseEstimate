package com.camera.pose.estimate.algorithm.coplanar_posit;

import com.camera.pose.estimate.math.Matrix33;
import com.camera.pose.estimate.math.Vector3;

public class PlanarPOSIT {

	private final float focalLength;

	private final Vector3[] modelPoints;
	private final Matrix33 modelVectors;
	private final Matrix33 modelPseudoInverse;
	private final Vector3 modelNormal;

	private Matrix33 alternateRotation;
	private Vector3 alternateTranslation;
	private float alternatePoseError;

	private Matrix33 bestRotation;
	private Vector3 bestTranslation;
	private float bestPoseError;

	public PlanarPOSIT(Vector3[] model, float focalLength) {
		if (model.length != 4)
			throw new RuntimeException("The model must have 4 points.");

		// XXX,something may be optimized here
		this.focalLength = focalLength;
		// this.focalLength = 960 - 60;
		modelPoints = Vector3.cloneArray(model);
		modelVectors = Matrix33.createFromRows(
				Vector3.subtract(model[1], model[0]),
				Vector3.subtract(model[2], model[0]),
				Vector3.subtract(model[3], model[0]));

		final Matrix33 resultU = new Matrix33(), resultV = new Matrix33();
		final Vector3 resultE = new Vector3();
		Matrix33.SVD(modelVectors, resultU, resultE, resultV);

		final Matrix33 tmp = Matrix33.multiply(resultV,
				Matrix33.createDiagonal(Vector3.inverse(resultE)));
		modelPseudoInverse = Matrix33
				.multiply(tmp, Matrix33.transpose(resultU));

		modelNormal = resultV.getColumn(resultE.minIndex());
	}

	public void estimatePose(Point[] points, float width, float height) {
		if (points.length != 4)
			throw new RuntimeException(
					"4 points must be be given for pose estimation.");
		points = picCoordToImgPlaneCoord(points, width, height);

		final Matrix33 resultRotation1 = new Matrix33(), resultRotation2 = new Matrix33();
		final Vector3 resultTranslation1 = new Vector3(), resultTranslation2 = new Vector3();

		pos(points, new Vector3(1), resultRotation1, resultRotation2,
				resultTranslation1, resultTranslation2);

		final float error1 = iterateCorrection(points, resultRotation1,
				resultTranslation1);
		final float error2 = iterateCorrection(points, resultRotation2,
				resultTranslation2);

		if (error1 < error2) {
			bestRotation = resultRotation1;
			bestTranslation = resultTranslation1;
			bestPoseError = error1;

			alternateRotation = resultRotation2;
			alternateTranslation = resultTranslation2;
			alternatePoseError = error2;
		} else {
			bestRotation = resultRotation2;
			bestTranslation = resultTranslation2;
			bestPoseError = error2;

			alternateRotation = resultRotation1;
			alternateTranslation = resultTranslation1;
			alternatePoseError = error1;
		}
	}

	private static final float ERROR_THRESHOLD = 2;

	private float iterateCorrection(Point[] points, Matrix33 rotationInit,
			Vector3 translationInit) {
		float prevError = Float.MAX_VALUE;
		float error = 0;
		Matrix33 rotation = rotationInit;
		Vector3 translation = translationInit;

		final Matrix33 resultRotation1 = new Matrix33(), resultRotation2 = new Matrix33();
		final Vector3 resultTranslation1 = new Vector3(), resultTranslation2 = new Vector3();

		for (int count = 0; count < 100; count++) {
			final Vector3 eps = Vector3.add(
					Vector3.divide((Matrix33.multiply(modelVectors,
							rotation.getRow(2))), translation.z), 1);

			pos(points, eps, resultRotation1, resultRotation2,
					resultTranslation1, resultTranslation2);

			final float error1 = calculateError(points, resultRotation1,
					resultTranslation1);
			final float error2 = calculateError(points, resultRotation2,
					resultTranslation2);

			if (error1 < error2) {
				rotation = resultRotation1;
				translation = resultTranslation1;
				error = error1;
			} else {
				rotation = resultRotation2;
				translation = resultTranslation2;
				error = error2;
			}

			if ((error <= ERROR_THRESHOLD) || (error > prevError))
				break;

			prevError = error;
		}

		rotationInit.fill(rotation);
		translationInit.fill(translation);
		return error;
	}

	private void pos(Point[] imagePoints, Vector3 eps,
			Matrix33 resultRotation1, Matrix33 resultRotation2,
			Vector3 resultTranslation1, Vector3 resultTranslation2) {
		final Vector3 XI = new Vector3(imagePoints[1].x, imagePoints[2].x,
				imagePoints[3].x);
		final Vector3 YI = new Vector3(imagePoints[1].y, imagePoints[2].y,
				imagePoints[3].y);

		final Vector3 imageXs = Vector3.subtract(Vector3.multiply(XI, eps),
				imagePoints[0].x);
		final Vector3 imageYs = Vector3.subtract(Vector3.multiply(YI, eps),
				imagePoints[0].y);

		final Vector3 I0Vector = Matrix33.multiply(modelPseudoInverse, imageXs);
		final Vector3 J0Vector = Matrix33.multiply(modelPseudoInverse, imageYs);

		final float j2i2dif = J0Vector.squaredLength()
				- I0Vector.squaredLength();
		final float ij = Vector3.dot(I0Vector, J0Vector);

		float r = 0, theta = 0;
		if (j2i2dif == 0) {
			theta = (float) ((-Math.PI / 2) * Math.signum(ij));
			r = (float) Math.sqrt(Math.abs(2 * ij));
		} else {
			r = (float) Math.sqrt(Math.sqrt(j2i2dif * j2i2dif + 4 * ij * ij));
			theta = (float) Math.atan(-2 * ij / j2i2dif);

			if (j2i2dif < 0)
				theta += (float) Math.PI;

			theta /= 2;
		}

		final float lambda = (float) (r * Math.cos(theta));
		final float mu = (float) (r * Math.sin(theta));

		{// solution1
			final Vector3 i = Vector3.add(I0Vector,
					Vector3.multiply(modelNormal, lambda));
			final Vector3 j = Vector3.add(J0Vector,
					Vector3.multiply(modelNormal, mu));
			calculateRotationAndTranslation(i, j, imagePoints[0],
					resultRotation1, resultTranslation1);
		}
		{// solution2
			final Vector3 iVector = Vector3.subtract(I0Vector,
					Vector3.multiply(modelNormal, lambda));
			final Vector3 jVector = Vector3.subtract(J0Vector,
					Vector3.multiply(modelNormal, mu));
			calculateRotationAndTranslation(iVector, jVector, imagePoints[0],
					resultRotation2, resultTranslation2);
		}
	}

	private void calculateRotationAndTranslation(Vector3 i, Vector3 j,
			Point imgPoint0, Matrix33 resultRotation, Vector3 resultTranslation) {
		final float iNorm = i.normalize();
		final float jNorm = j.normalize();
		final Vector3 k = Vector3.cross(i, j);
		final Matrix33 rotation = Matrix33.createFromRows(i, j, k);
		resultRotation.fill(rotation);

		final float scale = (iNorm + jNorm) / 2;
		final Vector3 temp = Matrix33.multiply(rotation, modelPoints[0]);
		resultTranslation.x = imgPoint0.x / scale - temp.x;
		resultTranslation.y = imgPoint0.y / scale - temp.y;
		resultTranslation.z = focalLength / scale;
	}

	private static Point[] picCoordToImgPlaneCoord(Point[] point, float width,
			float height) {
		final Point[] newPoints = new Point[point.length];
		for (int i = 0; i < newPoints.length; i++)
			newPoints[i] = new Point(point[i].x - width / 2, height / 2
					- point[i].y);
		return newPoints;
	}

	private float calculateError(Point[] imagePoints, Matrix33 rotation,
			Vector3 translation) {
		Vector3 v1 = Vector3.add(Matrix33.multiply(rotation, modelPoints[0]),
				translation);
		v1.x = v1.x * focalLength / v1.z;
		v1.y = v1.y * focalLength / v1.z;

		Vector3 v2 = Vector3.add(Matrix33.multiply(rotation, modelPoints[1]),
				translation);
		v2.x = v2.x * focalLength / v2.z;
		v2.y = v2.y * focalLength / v2.z;

		Vector3 v3 = Vector3.add(Matrix33.multiply(rotation, modelPoints[2]),
				translation);
		v3.x = v3.x * focalLength / v3.z;
		v3.y = v3.y * focalLength / v3.z;

		Vector3 v4 = Vector3.add(Matrix33.multiply(rotation, modelPoints[3]),
				translation);
		v4.x = v4.x * focalLength / v4.z;
		v4.y = v4.y * focalLength / v4.z;

		Point[] modeledPoints = new Point[] { new Point(v1.x, v1.y),
				new Point(v2.x, v2.y), new Point(v3.x, v3.y),
				new Point(v4.x, v4.y), };
		float ia1 = getAngleBetweenVectors(imagePoints[0], imagePoints[1],
				imagePoints[3]);
		float ia2 = getAngleBetweenVectors(imagePoints[1], imagePoints[2],
				imagePoints[0]);
		float ia3 = getAngleBetweenVectors(imagePoints[2], imagePoints[3],
				imagePoints[1]);
		float ia4 = getAngleBetweenVectors(imagePoints[3], imagePoints[0],
				imagePoints[2]);

		float ma1 = getAngleBetweenVectors(modeledPoints[0], modeledPoints[1],
				modeledPoints[3]);
		float ma2 = getAngleBetweenVectors(modeledPoints[1], modeledPoints[2],
				modeledPoints[0]);
		float ma3 = getAngleBetweenVectors(modeledPoints[2], modeledPoints[3],
				modeledPoints[1]);
		float ma4 = getAngleBetweenVectors(modeledPoints[3], modeledPoints[0],
				modeledPoints[2]);

		return (Math.abs(ia1 - ma1) + Math.abs(ia2 - ma2) + Math.abs(ia3 - ma3) + Math
				.abs(ia4 - ma4)) / 4;
		// double sumError = 0;
		// for (int i = 0; i < imagePoints.length; i++)
		// sumError += modeledPoints[i].squaredDistanceTo(imagePoints[i]);
		// return (float) (sumError / 4);
	}

	private static float getAngleBetweenVectors(Point startPoint,
			Point vector1end, Point vector2end) {
		float x1 = vector1end.x - startPoint.x;
		float y1 = vector1end.y - startPoint.y;

		float x2 = vector2end.x - startPoint.x;
		float y2 = vector2end.y - startPoint.y;

		return (float) (Math
				.cos((x1 * x2 + y1 * y2)
						/ (Math.sqrt(x1 * x1 + y1 * y1) * Math.sqrt(x2 * x2
								+ y2 * y2))) * 180.0 / Math.PI);
	}

	public Matrix33 getBestEstimatedRotation() {
		return bestRotation;
	}

	public Vector3 getBestEstimatedTranslation() {
		return bestTranslation;
	}

	public float getBestEstimationError() {
		return bestPoseError;
	}

	public Matrix33 getAlternateEstimatedRotation() {
		return alternateRotation;
	}

	public Vector3 getAlternateEstimatedTranslation() {
		return alternateTranslation;
	}

	public float getAlternateEstimationError() {
		return alternatePoseError;
	}
}
