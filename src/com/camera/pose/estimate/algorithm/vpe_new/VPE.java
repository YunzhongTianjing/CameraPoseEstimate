package com.camera.pose.estimate.algorithm.vpe_new;

import static java.lang.Math.abs;
import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import java.util.ArrayList;
import java.util.List;

public class VPE {
	public final double f;
	public final VPERealPoint Pc1, Pc2, Pc3, Pc4;
	public final VPERealPoint Pm1, Pm2, Pm3, Pm4;
	public final double[][] matrixN;

	private final double[] x;
	private final double[] y;
	private final double[][] s;
	private final double[][] C;

	/**
	 * @param imageX
	 *            real image x(reverse picture)
	 * @param imageY
	 *            real image y(reverse picture)
	 * @param s12
	 * @param s13
	 * @param s14
	 * @param s23
	 * @param s24
	 * @param s34
	 */
	public VPE(double[] imageX, double[] imageY, double s12, double s13,
			double s14, double s23, double s24, double s34, double picWidth,
			double picHeight) {
		for (int i = 0; i < imageX.length; i++) {
			final double x = imageX[i], y = imageY[i];
			imageX[i] = -(picHeight / 2 - y);
			imageY[i] = -(x - picWidth / 2);
		}

		this.x = new double[] { 0, imageX[0], imageX[1], imageX[2], imageX[3] };
		this.y = new double[] { 0, imageY[0], imageY[1], imageY[2], imageY[3] };
		this.s = to2dArray(s12, s13, s14, s23, s24, s34);
		final double[] A = A(s);
		final double[] B = B(x, y);
		this.C = C(A, B);
		this.f = f();

		this.Pc1 = Pc(1);
		this.Pc2 = Pc(2);
		this.Pc3 = Pc(3);
		this.Pc4 = Pc(4);

		this.Pm1 = Pm(1);
		this.Pm2 = Pm(2);
		this.Pm3 = Pm(3);
		this.Pm4 = Pm(4);

		this.matrixN = calculateMatrixN();

		System.out.println("__________________result__________________");
		System.out.println("Pc1 is " + Pc1);
		System.out.println("Pc2 is " + Pc2);
		System.out.println("Pc3 is " + Pc3);
		System.out.println("Pc4 is " + Pc4);
		System.out.println();
		System.out.println("Pm1 is " + Pm1);
		System.out.println("Pm2 is " + Pm2);
		System.out.println("Pm3 is " + Pm3);
		System.out.println("Pm4 is " + Pm4);
		System.out.println();
		System.out.println("matrixN is:");
		// MatrixUtils.printMatrix(matrixN);
		// T = NM
	}

	private double[][] calculateMatrixN() {
		final double denominator = Pm3.x * Pm4.y - Pm4.x * Pm3.y;
		/************* N11、N21、N31 ***************/
		double numerator = Pm4.y * (Pc3.x - Pc1.x) - Pm3.y * (Pc4.x - Pc1.x);
		final double N11 = numerator / denominator;

		numerator = Pm4.y * (Pc3.y - Pc1.y) - Pm3.y * (Pc4.y - Pc1.y);
		final double N21 = numerator / denominator;

		numerator = Pm4.y * (Pc3.z - Pc1.z) - Pm3.y * (Pc4.z - Pc1.z);
		final double N31 = numerator / denominator;

		/************* N12、N22、N32 ***************/
		numerator = Pm3.x * (Pc4.x - Pc1.x) - Pm4.x * (Pc3.x - Pc1.x);
		final double N12 = numerator / denominator;

		numerator = Pm3.x * (Pc4.y - Pc1.y) - Pm4.x * (Pc3.y - Pc1.y);
		final double N22 = numerator / denominator;

		numerator = Pm3.x * (Pc4.z - Pc1.z) - Pm4.x * (Pc3.z - Pc1.z);
		final double N32 = numerator / denominator;

		/************* N14、N24、N34 ***************/
		final double N14 = Pc1.x;
		final double N24 = Pc1.y;
		final double N34 = Pc1.z;

		/************* N13、N23、N33 ***************/
		final double N13 = N21 * N32 - N22 * N31;
		final double N23 = N12 * N31 - N11 * N32;
		final double N33 = N11 * N22 - N12 * N21;

		// return new double[] {
		// //
		// N11, N21, N31, 0, //
		// N12, N22, N32, 0, //
		// N13, N23, N33, 0, //
		// N14, N24, N34, 1,//
		// };
		return new double[][] {
				//
				{ N11, N21, N31, 0 }, //
				{ N12, N22, N32, 0 }, //
				{ N13, N23, N33, 0 }, //
				{ N14, N24, N34, 1 },//
		};
	}

	private VPERealPoint Pm(int n) {
		switch (n) {
		case 1:
			return new VPERealPoint(0, 0, 0);

		case 2:
			return new VPERealPoint(s[1][2], 0, 0);

		case 3:
			return new VPERealPoint(s[1][3] * cos(angleTheta()), s[1][3]
					* sin(angleTheta()), 0);

		case 4:
			return new VPERealPoint(s[1][4] * cos(angleFi()), s[1][4]
					* sin(angleFi()), 0);
		default:
			throw new RuntimeException("invalid n for " + n);
		}
	}

	private double angleTheta() {
		return acos(new VPEVector3D(Pc1, Pc2).dot(new VPEVector3D(Pc1, Pc3))
				/ (s[1][2] * s[1][3]));
	}

	private double angleFi() {
		return acos(new VPEVector3D(Pc1, Pc2).dot(new VPEVector3D(Pc1, Pc4))
				/ (s[1][2] * s[1][4]));
	}

	private VPERealPoint Pc(int n) {
		final S[] set = S.SET;
		final List<VPERealPoint> points = new ArrayList<VPERealPoint>();
		for (S s : set) {
			VPERealPoint pc = Pc(n, s.i, s.j, s.k);
			if (Double.isNaN(pc.x) || Double.isNaN(pc.y) || Double.isNaN(pc.z))
				continue;
			points.add(pc);
			System.out.println("Pc" + n + "." + s + "." + pc);
		}
		return VPERealPoint.getMidiant(points);
	}

	private VPERealPoint Pc(int n, int i, int j, int k) {
		final double _x = -x[n] * L(i, j, k);
		final double _y = -y[n] * L(i, j, k);
		final double _z = W(i, j, k) / G(i, j, k) + W(i, j, k)
				/ (C[i][n] * E(i, j, k));
		return new VPERealPoint(C[i][n] * _x, C[i][n] * _y, C[i][n] * _z);
	}

	private double E(int i, int j, int k) {
		final double left = pow2(s[i][j]) * pow2(1 - C[i][k]);
		final double right = pow2(s[i][k]) * pow2(1 - C[i][j]);
		return sqrt(abs(left - right));
	}

	private double G(int i, int j, int k) {
		final double left = H2(i, j) * pow2(1 - C[i][k]);
		final double right = H2(i, k) * pow2(1 - C[i][j]);
		return sqrt(abs(left - right));
	}

	private double W(int i, int j, int k) {
		return sqrt(abs(H2(i, j) * pow2(s[i][k]) - H2(i, k) * pow2(s[i][j])));
	}

	private double L(int i, int j, int k) {
		return E(i, j, k) / G(i, j, k);
	}

	private double f() {
		final S[] set = S.SET;
		double f = 0;
		int counts = 0;
		for (S s : set) {
			double tmpF = f(s.i, s.j, s.k);
			if (Double.isNaN(tmpF))
				continue;
			f += tmpF;
			counts++;
		}
		return f / counts;
	}

	private double f(int i, int j, int k) {
		final double numerator = pow2(s[i][k]) * H2(i, j) - pow2(s[i][j])
				* H2(i, k);
		final double denominator = pow2(s[i][j]) * pow2(1 - C[i][k])
				- pow2(s[i][k]) * pow2(1 - C[i][j]);
		return sqrt(numerator / denominator);
	}

	private double H2(int i, int j) {
		return pow2(x[i] - C[i][j] * x[j]) + pow2(y[i] - C[i][j] * y[j]);
	}

	private double[][] C(double[] A, double[] B) {
		final double C12 = B[3] * A[4] / (A[3] * B[4]);
		final double C13 = B[2] * A[4] / (A[2] * B[4]);
		final double C14 = B[1] * A[4] / (A[1] * B[4]);
		final double C23 = B[2] * A[3] / (A[2] * B[3]);
		final double C24 = B[1] * A[3] / (A[1] * B[3]);
		final double C34 = B[1] * A[2] / (A[1] * B[2]);
		final double C21 = 1 / C12;
		final double C31 = 1 / C13;
		final double C32 = 1 / C23;
		final double C41 = 1 / C14;
		final double C42 = 1 / C24;
		final double C43 = 1 / C34;
		final double C11 = 0, C22 = 0, C33 = 0, C44 = 0;
		return new double[][] {
				//
				{},//
				{ 0, C11, C12, C13, C14 },//
				{ 0, C21, C22, C23, C24 },//
				{ 0, C31, C32, C33, C34 },//
				{ 0, C41, C42, C43, C44 },//
		};
	}

	// tested by 3 , 4 , 5
	private double[] A(double[][] s) {
		double left = pow2(s[1][2]) + pow2(s[1][3]) + pow2(s[2][3]);
		left = pow2(left);
		double right = pow4(s[1][2]) + pow4(s[1][3]) + pow4(s[2][3]);
		right *= 2;
		final double A1 = sqrt(left - right) / 4;

		left = pow2(s[1][2]) + pow2(s[1][4]) + pow2(s[2][4]);
		left = pow2(left);
		right = pow4(s[1][2]) + pow4(s[1][4]) + pow4(s[2][4]);
		right *= 2;
		final double A2 = sqrt(left - right) / 4;

		left = pow2(s[1][3]) + pow2(s[1][4]) + pow2(s[3][4]);
		left = pow2(left);
		right = pow4(s[1][3]) + pow4(s[1][4]) + pow4(s[3][4]);
		right *= 2;
		final double A3 = sqrt(left - right) / 4;

		left = pow2(s[2][3]) + pow2(s[2][4]) + pow2(s[3][4]);
		left = pow2(left);
		right = pow4(s[2][3]) + pow4(s[2][4]) + pow4(s[3][4]);
		right *= 2;
		final double A4 = sqrt(left - right) / 4;

		return new double[] { 0, A1, A2, A3, A4 };
	}

	private double[] B(double[] x, double[] y) {
		final double B1 = x[1] * (y[3] - y[2]) + y[1] * (x[2] - x[3]) + y[2]
				* x[3] - x[2] * y[3];
		final double B2 = x[1] * (y[4] - y[2]) + y[1] * (x[2] - x[4]) + y[2]
				* x[4] - x[2] * y[4];
		final double B3 = x[1] * (y[4] - y[3]) + y[1] * (x[3] - x[4]) + y[3]
				* x[4] - x[3] * y[4];
		final double B4 = x[2] * (y[4] - y[3]) + y[2] * (x[3] - x[4]) + y[3]
				* x[4] - x[3] * y[4];
		return new double[] { 0, B1, B2, B3, B4 };
	}

	private double[][] to2dArray(double s12, double s13, double s14,
			double s23, double s24, double s34) {
		final double s21 = s12;
		final double s31 = s13;
		final double s41 = s14;
		final double s32 = s23;
		final double s42 = s24;
		final double s43 = s34;
		final double s11 = 0, s22 = 0, s33 = 0, s44 = 0;
		return new double[][] {
				//
				{},//
				{ 0, s11, s12, s13, s14 },//
				{ 0, s21, s22, s23, s24 },//
				{ 0, s31, s32, s33, s34 },//
				{ 0, s41, s42, s43, s44 },//
		};
	}

	private static double pow2(double x) {
		return x * x;
	}

	private static double pow4(double x) {
		return x * x * x * x;
	}

	public static void pixelToSensorCoordinate(double[] imageX, double[] imageY) {
		for (int i = 0; i < imageX.length; i++) {
			final double I = imageX[i];
			final double J = imageY[i];
			final double[] sensorCoordinate = pixelToSensorCoordinate(I, J);
			imageX[i] = sensorCoordinate[0];
			imageY[i] = sensorCoordinate[1];
		}
	}

	private static double[] pixelToSensorCoordinate(double I, double J) {
		final double x = a[0] + a[1] * I + a[2] * J + a[3] * I * I + a[4] * J
				* J + a[5] * I * J + a[6] * I * I * I + a[7] * J * J * J + a[8]
				* I * I * J + a[9] * I * J * J;

		final double y = b[0] + b[1] * I + b[2] * J + b[3] * I * I + b[4] * J
				* J + b[5] * I * J + b[6] * I * I * I + b[7] * J * J * J + b[8]
				* I * I * J + b[9] * I * J * J;

		return new double[] { x, y };
	}

	private static final double[] a = {
			//
			2.7514,//
			-1.8716E-2,//
			-2.5888E-4,//
			3.6307E-6,//
			1.2161E-6,//
			1.0169E-6,//
			-5.1562E-9,//
			-5.3668E-11,//
			3.2664E-10,//
			-4.7269E-9 //
	};

	private static final double[] b = {
			//
			-4.5047,//
			7.7559E-4,//
			1.7739E-2,//
			-1.7485E-6,//
			-2.0289E-6,//
			-2.1701E-6,//
			3.5263E-10,//
			3.9632E-9,//
			4.6629E-9,//
			-3.7592E-9 //
	};

	private static class S {
		private final int i, j, k;
		private final static S[] SET = generateSet();

		private S(int i, int j, int k) {
			this.i = i;
			this.j = j;
			this.k = k;
		}

		private static S[] generateSet() {
			return new S[] { new S(1, 2, 3), new S(1, 2, 4), new S(1, 3, 4),
					new S(2, 1, 3), new S(2, 1, 4), new S(2, 3, 4),
					new S(3, 1, 2), new S(3, 1, 4), new S(3, 2, 4),
					new S(4, 1, 2), new S(4, 1, 3), new S(4, 2, 3) };
		}

		@Override
		public String toString() {
			return "" + i + j + k;
		}
	}
}
