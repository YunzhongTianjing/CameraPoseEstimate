package com.camera.pose.estimate.algorithm.vpe_old;

public class VolumePoseEst2 {
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

	public VolumePoseEst2(float length, ImagePoint[] imagePoints,
			float picWidth, float picHeight) {
		imagePoints = transformCoordinate(imagePoints, picWidth, picHeight);
		this.imagePoints = getOriginSymmetryImagePoints(imagePoints);
		this.segments = calculateSegmentLength(length);
//		final float[] As = calculateAs(segments);
		final float[] Bs = calculateBs(imagePoints);
		this.Cs = calculateCs(Bs);

		this.meanF = calculateF();

		for (int i = 0; i < pointCoordsInCameraSys.length; i++) {
			this.pointCoordsInCameraSys[i] = calculatePInC(i);
			this.pointCoordsInCameraSys[i] = new RealPoint(
					this.pointCoordsInCameraSys[i].x,
					this.pointCoordsInCameraSys[i].y,
					this.pointCoordsInCameraSys[i].z - meanF);
		}

		this.matrixViewModel = this.matrixView = calculateViewMatrix5(length);
		test(Bs);
	}

	private float[] calculateAs(float[][] s) {
		double l = Math.pow(s[0][1], 2) + Math.pow(s[0][2], 2)
				+ Math.pow(s[1][2], 2);
		l = l * l;
		double r = Math.pow(s[0][1], 4) + Math.pow(s[0][2], 4)
				+ Math.pow(s[1][2], 4);
		r = 2 * r;
		final float A1 = (float) (Math.sqrt(l - r) / 4);

		l = Math.pow(s[0][1], 2) + Math.pow(s[0][3], 2) + Math.pow(s[1][3], 2);
		l = l * l;
		r = Math.pow(s[0][1], 4) + Math.pow(s[0][3], 4) + Math.pow(s[1][3], 4);
		r = 2 * r;
		final float A2 = (float) (Math.sqrt(l - r) / 4);

		l = Math.pow(s[0][2], 2) + Math.pow(s[0][3], 2) + Math.pow(s[2][3], 2);
		l = l * l;
		r = Math.pow(s[0][2], 4) + Math.pow(s[0][3], 4) + Math.pow(s[2][3], 4);
		r = 2 * r;
		final float A3 = (float) (Math.sqrt(l - r) / 4);

		l = Math.pow(s[1][2], 2) + Math.pow(s[1][3], 2) + Math.pow(s[2][3], 2);
		l = l * l;
		r = Math.pow(s[1][2], 4) + Math.pow(s[1][3], 4) + Math.pow(s[2][3], 4);
		r = 2 * r;
		final float A4 = (float) (Math.sqrt(l - r) / 4);

		return new float[] { A1, A2, A3, A4 };
	}

	private void test(float[] Bs) {
		int sum = 0;
		sum += Math.pow(Bs[2] * imagePoints[1].x - Bs[1] * imagePoints[2].x, 2);
		sum += Math.pow(Bs[2] * imagePoints[1].y - Bs[1] * imagePoints[2].y, 2);
		sum += meanF * meanF * (Bs[2] - Bs[1]) * (Bs[2] - Bs[1]);
		final double G1 = Math.sqrt(sum) / segments[1][2];

		sum = 0;
		sum += Math.pow(Bs[2] * imagePoints[1].x - Bs[0] * imagePoints[3].x, 2);
		sum += Math.pow(Bs[2] * imagePoints[1].y - Bs[0] * imagePoints[2].y, 2);
		sum += meanF * meanF * (Bs[2] - Bs[0]) * (Bs[2] - Bs[0]);
		final double G2 = Math.sqrt(sum) / segments[1][3];

		sum = 0;
		sum += Math.pow(Bs[1] * imagePoints[2].x - Bs[0] * imagePoints[3].x, 2);
		sum += Math.pow(Bs[1] * imagePoints[2].y - Bs[0] * imagePoints[3].y, 2);
		sum += meanF * meanF * (Bs[0] - Bs[1]) * (Bs[0] - Bs[1]);
		final double G3 = Math.sqrt(sum) / segments[2][3];

		sum = 0;
		sum += Math.pow(Bs[3] * imagePoints[0].x - Bs[2] * imagePoints[1].x, 2);
		sum += Math.pow(Bs[3] * imagePoints[0].y - Bs[2] * imagePoints[1].y, 2);
		sum += meanF * meanF * (Bs[3] - Bs[2]) * (Bs[3] - Bs[2]);
		final double G4 = Math.sqrt(sum) / segments[0][1];

		sum = 0;
		sum += Math.pow(Bs[3] * imagePoints[2].x - Bs[1] * imagePoints[2].x, 2);
		sum += Math.pow(Bs[3] * imagePoints[0].y - Bs[1] * imagePoints[2].y, 2);
		sum += meanF * meanF * (Bs[3] - Bs[1]) * (Bs[3] - Bs[1]);
		final double G5 = Math.sqrt(sum) / segments[0][2];

		System.out.println(G1 - G2);
	}

	private ImagePoint[] transformCoordinate(ImagePoint[] imagePoints,
			float picWidth, float picHeight) {
		final ImagePoint[] newImagePoint = new ImagePoint[imagePoints.length];
		for (int i = 0; i < newImagePoint.length; i++)
			newImagePoint[i] = new ImagePoint(picHeight / 2 - imagePoints[i].y,
					imagePoints[i].x - picWidth / 2);
		return newImagePoint;
	}

	private float[][] calculateViewMatrix2(float length) {
		final Vector3D iInC = new Vector3D(1, 0, 0);// x
		final Vector3D jInC = new Vector3D(0, 1, 0);// y
		final Vector3D kInC = new Vector3D(0, 0, 1);// z
		final Vector3D iInW = new Vector3D(pointCoordsInCameraSys[0],
				pointCoordsInCameraSys[1]).normalize();// x'
		final Vector3D kInW = new Vector3D(pointCoordsInCameraSys[0],
				pointCoordsInCameraSys[3]).normalize();// z'
		final Vector3D jInW = kInW.cross(iInW);// y'
		float tranX = (pointCoordsInCameraSys[0].x + pointCoordsInCameraSys[2].x) / 2;
		tranX += (pointCoordsInCameraSys[1].x + pointCoordsInCameraSys[3].x) / 2;
		tranX /= 2;
		float tranY = (pointCoordsInCameraSys[0].y + pointCoordsInCameraSys[2].y) / 2;
		tranY += (pointCoordsInCameraSys[1].y + pointCoordsInCameraSys[3].y) / 2;
		tranY /= 2;
		float tranZ = (pointCoordsInCameraSys[0].z + pointCoordsInCameraSys[2].z) / 2;
		tranZ += (pointCoordsInCameraSys[1].z + pointCoordsInCameraSys[3].z) / 2;
		tranZ /= 2;
		return new float[][] {
				{ iInC.dot(iInW), iInC.dot(jInW), iInC.dot(kInW), tranX },//
				{ jInC.dot(iInW), jInC.dot(jInW), jInC.dot(kInW), tranY },//
				{ kInC.dot(iInW), kInC.dot(jInW), kInC.dot(kInW), tranZ },//
				{ 0, 0, 0, 1 } //
		};
	}

	private float[][] calculateViewMatrix5(float length) {
		final Vector3D iInC = new Vector3D(1, 0, 0);// x
		final Vector3D jInC = new Vector3D(0, 1, 0);// y
		final Vector3D kInC = new Vector3D(0, 0, 1);// z
		final RealPoint p0 = new RealPoint(pointCoordsInCameraSys[0].y,
				pointCoordsInCameraSys[0].x, -pointCoordsInCameraSys[0].z);
		final RealPoint p1 = new RealPoint(pointCoordsInCameraSys[1].y,
				pointCoordsInCameraSys[1].x, -pointCoordsInCameraSys[1].z);
		final RealPoint p2 = new RealPoint(pointCoordsInCameraSys[2].y,
				pointCoordsInCameraSys[2].x, -pointCoordsInCameraSys[2].z);
		final RealPoint p3 = new RealPoint(pointCoordsInCameraSys[3].y,
				pointCoordsInCameraSys[3].x, -pointCoordsInCameraSys[3].z);

		final Vector3D iInW = new Vector3D(p0, p1).normalize();// x'
		final Vector3D jInW = new Vector3D(p3, p0).normalize();// y'
		final Vector3D kInW = jInW.cross(iInW);// z'

		final Vector3D vecTran = calculateMeanTranslateVector(p0, p1, p2, p3,
				length);

		return new float[][] {
				{ iInC.dot(iInW), iInC.dot(jInW), -iInC.dot(kInW), vecTran.x },//
				{ jInC.dot(iInW), jInC.dot(jInW), -jInC.dot(kInW), vecTran.y },//
				{ kInC.dot(iInW), kInC.dot(jInW), -kInC.dot(kInW), vecTran.z },//
				{ 0, 0, 0, 1 } //
		};
	}

	private Vector3D calculateMeanTranslateVector(RealPoint p0, RealPoint p1,
			RealPoint p2, RealPoint p3, float length) {
		final RealPoint p0Model = new RealPoint(-length / 2, length / 2, 0);
		final RealPoint p1Model = new RealPoint(length / 2, length / 2, 0);
		final RealPoint p2Model = new RealPoint(length / 2, -length / 2, 0);
		final RealPoint p3Model = new RealPoint(-length / 2, -length / 2, 0);
		return Vector3D.average(
		//
				new Vector3D(p0Model, p0), //
				new Vector3D(p1Model, p1), //
				new Vector3D(p2Model, p2),//
				new Vector3D(p3Model, p3)
		//
				);
	}

	private float[][] calculateViewMatrix3(float length) {
		final Vector3D iInC = new Vector3D(1, 0, 0);// x
		final Vector3D jInC = new Vector3D(0, 1, 0);// y
		final Vector3D kInC = new Vector3D(0, 0, 1);// z
		final Vector3D iInW = new Vector3D(pointCoordsInCameraSys[3],
				pointCoordsInCameraSys[1]).normalize();// x'
		final Vector3D jInW = new Vector3D(pointCoordsInCameraSys[3],
				pointCoordsInCameraSys[2]).normalize();// y'
		final Vector3D kInW = jInW.cross(iInW);// y'
		float tranX = (pointCoordsInCameraSys[0].x + pointCoordsInCameraSys[2].x) / 2;
		tranX += (pointCoordsInCameraSys[1].x + pointCoordsInCameraSys[3].x) / 2;
		tranX /= 2;
		float tranY = (pointCoordsInCameraSys[0].y + pointCoordsInCameraSys[2].y) / 2;
		tranY += (pointCoordsInCameraSys[1].y + pointCoordsInCameraSys[3].y) / 2;
		tranY /= 2;
		float tranZ = (pointCoordsInCameraSys[0].z + pointCoordsInCameraSys[2].z) / 2;
		tranZ += (pointCoordsInCameraSys[1].z + pointCoordsInCameraSys[3].z) / 2;
		tranZ /= 2;
		return new float[][] {
				{ iInC.dot(iInW), iInC.dot(jInW), iInC.dot(kInW), tranX },//
				{ jInC.dot(iInW), jInC.dot(jInW), jInC.dot(kInW), tranY },//
				{ kInC.dot(iInW), kInC.dot(jInW), kInC.dot(kInW), tranZ },//
				{ 0, 0, 0, 1 } //
		};
	}

	// private void testf() {
	// final float R = (float) Math.sqrt(squareH(0, 1) + meanF * meanF
	// * (1 - Cs[0][1]) * (1 - Cs[0][1]));
	// float x = -imagePoints[0].x * segments[0][1] / R;
	//
	// float z = (float) (meanF * (segments[0][1] / R + 1));
	// System.out.println("x by f:" + x);
	// System.out.println("z by f:" + z);
	// }

	private ImagePoint[] getOriginSymmetryImagePoints(
			ImagePoint[] oldImagePoints) {
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
		Bs[0] = imagePoints[0].x * (imagePoints[2].y - imagePoints[1].y)
				+ imagePoints[0].y * (imagePoints[1].x - imagePoints[2].x)
				+ imagePoints[1].y * imagePoints[2].x - imagePoints[1].x
				* imagePoints[2].y;

		Bs[1] = imagePoints[0].x * (imagePoints[3].y - imagePoints[1].y)
				+ imagePoints[0].y * (imagePoints[1].x - imagePoints[3].x)
				+ imagePoints[1].y * imagePoints[3].x - imagePoints[1].x
				* imagePoints[3].y;

		Bs[2] = imagePoints[0].x * (imagePoints[3].y - imagePoints[2].y)
				+ imagePoints[0].y * (imagePoints[2].x - imagePoints[3].x)
				+ imagePoints[2].y * imagePoints[3].x - imagePoints[2].x
				* imagePoints[3].y;

		Bs[3] = imagePoints[1].x * (imagePoints[3].y - imagePoints[2].y)
				+ imagePoints[1].y * (imagePoints[2].x - imagePoints[3].x)
				+ imagePoints[2].y * imagePoints[3].x - imagePoints[2].x
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
		return (imagePoints[i].x - Cs[i][j] * imagePoints[j].x)
				* (imagePoints[i].x - Cs[i][j] * imagePoints[j].x)
				+ (imagePoints[i].y - Cs[i][j] * imagePoints[j].y)
				* (imagePoints[i].y - Cs[i][j] * imagePoints[j].y);
	}

	private float f(int i, int j, int k) {
		final float numerator = segments[i][k] * segments[i][k] * squareH(i, j)
				- segments[i][j] * segments[i][j] * squareH(i, k);
		final float denominator = segments[i][j] * segments[i][j]
				* (1 - Cs[i][k]) * (1 - Cs[i][k]) - segments[i][k]
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
		final float tmp = segments[i][j] * segments[i][j] * (1 - Cs[i][k])
				* (1 - Cs[i][k]) - segments[i][k] * segments[i][k]
				* (1 - Cs[i][j]) * (1 - Cs[i][j]);
		return (float) Math.sqrt(Math.abs(tmp));
	}

	private float g(int i, int j, int k) {
		final float tmp = squareH(i, j) * (1 - Cs[i][k]) * (1 - Cs[i][k])
				- squareH(i, k) * (1 - Cs[i][j]) * (1 - Cs[i][j]);
		return (float) Math.sqrt(Math.abs(tmp));
	}

	private float w(int i, int j, int k) {
		final float tmp = squareH(i, j) * segments[i][k] * segments[i][k]
				- squareH(i, k) * segments[i][j] * segments[i][j];
		return (float) Math.sqrt(Math.abs(tmp));
	}

	private float l(int i, int j, int k) {
		return e(i, j, k) / g(i, j, k);
	}

	private RealPoint calculatePInC(int n) {
		final IJK[] set = IJK.generateSet();
		float x = 0;
		float y = 0;
		float z = 0;
		int counts = 0;
		for (IJK ijk : set) {
			if (0 == Cs[ijk.i][n])
				continue;
			final float Lijk = l(ijk.i, ijk.j, ijk.k);
			x += -Cs[ijk.i][n] * imagePoints[n].x * Lijk;
			y += -Cs[ijk.i][n] * imagePoints[n].y * Lijk;

			final float Wijk = w(ijk.i, ijk.j, ijk.k);
			final float Gijk = g(ijk.i, ijk.j, ijk.k);
			z += Cs[ijk.i][n]
					* (Wijk / Gijk + Wijk
							/ (Cs[ijk.i][n] * e(ijk.i, ijk.j, ijk.k)));
			counts++;
		}

		return new RealPoint(x / counts, y / counts, z / counts);
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
