package com.camera.pose.estimate.algorithm.vpe_old;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class VolumePoseEst {
	private final ImagePoint[] imagePoints;
	private final float[][] segments;
	private final float[][] Cs;
	public final float meanF;

	public final RealPoint[] pointCoordsInCameraSys = new RealPoint[4];

	@SuppressWarnings("unused")
	private final float[][] matrixView;// model coordinate system as world
										// coordinate system , then it will be
										// view_model_matrix

	public final float[][] matrixViewModel;

	public VolumePoseEst(float length, ImagePoint[] imagePoints, float picWidth, float picHeight) {
		imagePoints = transformCoordinate(imagePoints, picWidth, picHeight);
		imagePoints = getOriginSymmetryImagePoints(imagePoints);
		this.imagePoints = imagePoints;
		this.segments = calculateSegmentLength(length);
		final float[] Bs = calculateBs(imagePoints);
		this.Cs = calculateCs(Bs);

		this.meanF = calculateF();

		for (int i = 0; i < pointCoordsInCameraSys.length; i++) {
			this.pointCoordsInCameraSys[i] = calculatePInC2(i);
			// change origin from image center to C(focus point)
			this.pointCoordsInCameraSys[i] = new RealPoint(this.pointCoordsInCameraSys[i].x, this.pointCoordsInCameraSys[i].y,
					this.pointCoordsInCameraSys[i].z - meanF);
		}

		this.matrixViewModel = this.matrixView = calculateViewMatrix5(length);
		testProjectPoints();
	}

	private void testProjectPoints() {
		for (int i = 0; i < pointCoordsInCameraSys.length; i++) {
			final RealPoint point = pointCoordsInCameraSys[i];
			final float x = point.x * (meanF / point.z);
			final float y = point.y * (meanF / point.z);
			final ImagePoint myImagePoint = new ImagePoint(x, y);
			System.out.println(myImagePoint);
		}
	}

	private ImagePoint[] transformCoordinate(ImagePoint[] imagePoints, float picWidth, float picHeight) {
		final ImagePoint[] newImagePoint = new ImagePoint[imagePoints.length];
		for (int i = 0; i < newImagePoint.length; i++)
			newImagePoint[i] = new ImagePoint(picHeight / 2 - imagePoints[i].y, imagePoints[i].x - picWidth / 2);
		return newImagePoint;
	}

	private float[][] calculateViewMatrix5(float length) {
		final Vector3D iInC = new Vector3D(1, 0, 0);// x
		final Vector3D jInC = new Vector3D(0, 1, 0);// y
		final Vector3D kInC = new Vector3D(0, 0, 1);// z
		final RealPoint p0 = new RealPoint(pointCoordsInCameraSys[0].y, pointCoordsInCameraSys[0].x, -pointCoordsInCameraSys[0].z);
		final RealPoint p1 = new RealPoint(pointCoordsInCameraSys[1].y, pointCoordsInCameraSys[1].x, -pointCoordsInCameraSys[1].z);
		final RealPoint p2 = new RealPoint(pointCoordsInCameraSys[2].y, pointCoordsInCameraSys[2].x, -pointCoordsInCameraSys[2].z);
		final RealPoint p3 = new RealPoint(pointCoordsInCameraSys[3].y, pointCoordsInCameraSys[3].x, -pointCoordsInCameraSys[3].z);

		final Vector3D iInW = new Vector3D(p0, p1).normalize();// x'
		final Vector3D jInW = new Vector3D(p3, p0).normalize();// y'
		final Vector3D kInW = jInW.cross(iInW);// z'

		final Vector3D vecTran = calculateMeanTranslateVector(p0, p1, p2, p3, length);

		return new float[][] {
				//
				{ iInC.dot(iInW), iInC.dot(jInW), -iInC.dot(kInW), vecTran.x },//
				{ jInC.dot(iInW), jInC.dot(jInW), -jInC.dot(kInW), vecTran.y },//
				{ kInC.dot(iInW), kInC.dot(jInW), -kInC.dot(kInW), vecTran.z },//
				{ 0, 0, 0, 1 } //
		};
	}

	private Vector3D calculateMeanTranslateVector(RealPoint p0, RealPoint p1, RealPoint p2, RealPoint p3, float length) {
		final RealPoint p0Model = new RealPoint(-length / 2, length / 2, 0);
		final RealPoint p1Model = new RealPoint(length / 2, length / 2, 0);
		final RealPoint p2Model = new RealPoint(length / 2, -length / 2, 0);
		final RealPoint p3Model = new RealPoint(-length / 2, -length / 2, 0);
		return Vector3D.average(
		//
				new Vector3D(p0Model, p0), //
				new Vector3D(p1Model, p1), //
				new Vector3D(p2Model, p2),//
				new Vector3D(p3Model, p3));
	}

	private ImagePoint[] getOriginSymmetryImagePoints(ImagePoint[] oldImagePoints) {
		final ImagePoint[] newImagePoints = new ImagePoint[oldImagePoints.length];
		newImagePoints[0] = oldImagePoints[0].originOfSymmetry();
		newImagePoints[1] = oldImagePoints[1].originOfSymmetry();
		newImagePoints[2] = oldImagePoints[2].originOfSymmetry();
		newImagePoints[3] = oldImagePoints[3].originOfSymmetry();
		return newImagePoints;
	}

	private float[][] calculateSegmentLength(float length) {
		final float[][] res = new float[4][4];
		res[0][0] = 0;
		res[0][1] = length;
		res[0][2] = (float) (Math.sqrt(2) * length);
		res[0][3] = length;

		res[1][0] = length;
		res[1][1] = 0;
		res[1][2] = length;
		res[1][3] = (float) (Math.sqrt(2) * length);

		res[2][0] = (float) (Math.sqrt(2) * length);
		res[2][1] = length;
		res[2][2] = 0;
		res[2][3] = length;

		res[3][0] = length;
		res[3][1] = (float) (Math.sqrt(2) * length);
		res[3][2] = length;
		res[3][3] = 0;
		return res;
	}

	private float[] calculateBs(ImagePoint[] imagePoints) {
		final float[] Bs = new float[4];
		Bs[0] = imagePoints[0].x * (imagePoints[2].y - imagePoints[1].y) + imagePoints[0].y
				* (imagePoints[1].x - imagePoints[2].x) + imagePoints[1].y * imagePoints[2].x - imagePoints[1].x
				* imagePoints[2].y;

		Bs[1] = imagePoints[0].x * (imagePoints[3].y - imagePoints[1].y) + imagePoints[0].y
				* (imagePoints[1].x - imagePoints[3].x) + imagePoints[1].y * imagePoints[3].x - imagePoints[1].x
				* imagePoints[3].y;

		Bs[2] = imagePoints[0].x * (imagePoints[3].y - imagePoints[2].y) + imagePoints[0].y
				* (imagePoints[2].x - imagePoints[3].x) + imagePoints[2].y * imagePoints[3].x - imagePoints[2].x
				* imagePoints[3].y;

		Bs[3] = imagePoints[1].x * (imagePoints[3].y - imagePoints[2].y) + imagePoints[1].y
				* (imagePoints[2].x - imagePoints[3].x) + imagePoints[2].y * imagePoints[3].x - imagePoints[2].x
				* imagePoints[3].y;
		return Bs;
	}

	private float[][] calculateCs(float[] Bs) {
		final float[][] Cs = new float[4][4];
		Cs[0][0] = 0;
		Cs[0][1] = Bs[2] / Bs[3];
		Cs[0][2] = Bs[1] / Bs[3];
		Cs[0][3] = Bs[0] / Bs[3];

		Cs[1][0] = 1 / Cs[0][1];
		Cs[1][1] = 0;
		Cs[1][2] = Bs[1] / Bs[2];
		Cs[1][3] = Bs[0] / Bs[2];

		Cs[2][0] = 1 / Cs[0][2];
		Cs[2][1] = 1 / Cs[1][2];
		Cs[2][2] = 0;
		Cs[2][3] = Bs[0] / Bs[1];

		Cs[3][0] = 1 / Cs[0][3];
		Cs[3][1] = 1 / Cs[1][3];
		Cs[3][2] = 1 / Cs[2][3];
		Cs[3][3] = 0;
		return Cs;
	}

	private float squareH(int i, int j) {
		return (imagePoints[i].x - Cs[i][j] * imagePoints[j].x) * (imagePoints[i].x - Cs[i][j] * imagePoints[j].x)
				+ (imagePoints[i].y - Cs[i][j] * imagePoints[j].y) * (imagePoints[i].y - Cs[i][j] * imagePoints[j].y);
	}

	private float f(int i, int j, int k) {
		final float numerator = segments[i][k] * segments[i][k] * squareH(i, j) - segments[i][j] * segments[i][j] * squareH(i, k);
		final float denominator = segments[i][j] * segments[i][j] * (1 - Cs[i][k]) * (1 - Cs[i][k]) - segments[i][k]
				* segments[i][k] * (1 - Cs[i][j]) * (1 - Cs[i][j]);
		if (numerator * denominator <= 0)
			return -1;
		return (float) Math.sqrt(numerator / denominator);
	}

	private float calculateF() {
		final IJK[] set = IJK.generateSet();
		float sumf = 0;
		int counts = 0;
		for (IJK ijk : set) {
			final float f = f(ijk.i, ijk.j, ijk.k);
			if (-1 != f) {
				sumf += f;
				counts++;
			}
		}
		return counts > 0 ? sumf / counts : 0;
	}

	private float e(int i, int j, int k) {
		final float tmp = segments[i][j] * segments[i][j] * (1 - Cs[i][k]) * (1 - Cs[i][k]) - segments[i][k] * segments[i][k]
				* (1 - Cs[i][j]) * (1 - Cs[i][j]);
		return (float) Math.sqrt(Math.abs(tmp));
	}

	private float g(int i, int j, int k) {
		final float tmp = squareH(i, j) * (1 - Cs[i][k]) * (1 - Cs[i][k]) - squareH(i, k) * (1 - Cs[i][j]) * (1 - Cs[i][j]);
		return (float) Math.sqrt(Math.abs(tmp));
	}

	private float w(int i, int j, int k) {
		final float tmp = squareH(i, j) * segments[i][k] * segments[i][k] - squareH(i, k) * segments[i][j] * segments[i][j];
		return (float) Math.sqrt(Math.abs(tmp));
	}

	private float l(int i, int j, int k) {
		return e(i, j, k) / g(i, j, k);
	}

	/**
	 * image center as origin
	 * 
	 * @param n
	 * @return
	 */
	private RealPoint calculatePInC2(int n) {
		final IJK[] set = IJK.generateSet();
		List<RealPoint> points = new ArrayList<RealPoint>();
		for (IJK ijk : set) {
			if (0 == Cs[ijk.i][n])
				continue;
			final float Lijk = l(ijk.i, ijk.j, ijk.k);
			final float x = -Cs[ijk.i][n] * imagePoints[n].x * Lijk;
			final float y = -Cs[ijk.i][n] * imagePoints[n].y * Lijk;

			final float Wijk = w(ijk.i, ijk.j, ijk.k);
			final float Gijk = g(ijk.i, ijk.j, ijk.k);
			final float z = Cs[ijk.i][n] * (Wijk / Gijk + Wijk / (Cs[ijk.i][n] * e(ijk.i, ijk.j, ijk.k)));
			points.add(new RealPoint(x, y, z));
		}
		Collections.sort(points);
		return points.get(points.size() / 2);
	}

	@Override
	public String toString() {
		String res = "f:" + meanF + "\n";
		for (int i = 0; i < pointCoordsInCameraSys.length; i++) {
			res += "p[" + i + "]->" + pointCoordsInCameraSys[i].toString();
			res += "\n";
		}
		return res;
	}

}
