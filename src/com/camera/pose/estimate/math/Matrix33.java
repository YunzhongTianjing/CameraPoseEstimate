package com.camera.pose.estimate.math;

public class Matrix33 {
	public float v00, v01, v02, v10, v11, v12, v20, v21, v22;

	public void fill(Matrix33 matrix) {
		this.v00 = matrix.v00;
		this.v01 = matrix.v01;
		this.v02 = matrix.v02;

		this.v10 = matrix.v10;
		this.v11 = matrix.v11;
		this.v12 = matrix.v12;

		this.v20 = matrix.v20;
		this.v21 = matrix.v21;
		this.v22 = matrix.v22;
	}

	public float determinant() {
		return v00 * v11 * v22 + v01 * v12 * v20 + v02 * v10 * v21 - v00 * v12
				* v21 - v01 * v10 * v22 - v02 * v11 * v20;
	}

	/**
	 * Y、X、Z
	 * 
	 * @param resultYawPitchRoll
	 */
	public void getYawPitchRoll(float[] resultYawPitchRoll) {
		final float yaw = (float) Math.atan2(v02, v22);
		final float pitch = (float) Math.asin(-v12);
		final float roll = (float) Math.atan2(v10, v11);
		resultYawPitchRoll[0] = (float) (yaw * 180 / Math.PI);
		resultYawPitchRoll[1] = (float) (pitch * 180 / Math.PI);
		resultYawPitchRoll[2] = (float) (roll * 180 / Math.PI);
	}

	public Vector3 getRow(int index) {
		if ((index < 0) || (index > 2))
			throw new RuntimeException("Invalid row index was specified.");

		return (index == 0) ? new Vector3(v00, v01, v02)
				: (index == 1) ? new Vector3(v10, v11, v12) : new Vector3(v20,
						v21, v22);
	}

	public Vector3 getColumn(int index) {
		if ((index < 0) || (index > 2))
			throw new RuntimeException("Invalid column index was specified.");

		return (index == 0) ? new Vector3(v00, v10, v20)
				: (index == 1) ? new Vector3(v01, v11, v21) : new Vector3(v02,
						v12, v22);
	}

	public static Matrix33 createFromRows(Vector3 row0, Vector3 row1,
			Vector3 row2) {
		Matrix33 m = new Matrix33();

		m.v00 = row0.x;
		m.v01 = row0.y;
		m.v02 = row0.z;

		m.v10 = row1.x;
		m.v11 = row1.y;
		m.v12 = row1.z;

		m.v20 = row2.x;
		m.v21 = row2.y;
		m.v22 = row2.z;

		return m;
	}

	public static Matrix33 createDiagonal(Vector3 vector) {
		Matrix33 m = new Matrix33();

		m.v00 = vector.x;
		m.v11 = vector.y;
		m.v22 = vector.z;

		return m;
	}

	public static Matrix33 multiply(Matrix33 matrix1, Matrix33 matrix2) {
		Matrix33 m = new Matrix33();

		m.v00 = matrix1.v00 * matrix2.v00 + matrix1.v01 * matrix2.v10
				+ matrix1.v02 * matrix2.v20;
		m.v01 = matrix1.v00 * matrix2.v01 + matrix1.v01 * matrix2.v11
				+ matrix1.v02 * matrix2.v21;
		m.v02 = matrix1.v00 * matrix2.v02 + matrix1.v01 * matrix2.v12
				+ matrix1.v02 * matrix2.v22;

		m.v10 = matrix1.v10 * matrix2.v00 + matrix1.v11 * matrix2.v10
				+ matrix1.v12 * matrix2.v20;
		m.v11 = matrix1.v10 * matrix2.v01 + matrix1.v11 * matrix2.v11
				+ matrix1.v12 * matrix2.v21;
		m.v12 = matrix1.v10 * matrix2.v02 + matrix1.v11 * matrix2.v12
				+ matrix1.v12 * matrix2.v22;

		m.v20 = matrix1.v20 * matrix2.v00 + matrix1.v21 * matrix2.v10
				+ matrix1.v22 * matrix2.v20;
		m.v21 = matrix1.v20 * matrix2.v01 + matrix1.v21 * matrix2.v11
				+ matrix1.v22 * matrix2.v21;
		m.v22 = matrix1.v20 * matrix2.v02 + matrix1.v21 * matrix2.v12
				+ matrix1.v22 * matrix2.v22;

		return m;
	}

	public static Vector3 multiply(Matrix33 matrix, Vector3 vector) {
		return new Vector3(matrix.v00 * vector.x + matrix.v01 * vector.y
				+ matrix.v02 * vector.z, matrix.v10 * vector.x + matrix.v11
				* vector.y + matrix.v12 * vector.z, matrix.v20 * vector.x
				+ matrix.v21 * vector.y + matrix.v22 * vector.z);
	}

	public static Matrix33 transpose(Matrix33 matrix) {
		Matrix33 m = new Matrix33();

		m.v00 = matrix.v00;
		m.v01 = matrix.v10;
		m.v02 = matrix.v20;

		m.v10 = matrix.v01;
		m.v11 = matrix.v11;
		m.v12 = matrix.v21;

		m.v20 = matrix.v02;
		m.v21 = matrix.v12;
		m.v22 = matrix.v22;

		return m;
	}

	public static void SVD(Matrix33 matrix, Matrix33 resultU, Vector3 resultE,
			Matrix33 resultV) {
		double[][] uArray = new double[][] {
				{ matrix.v00, matrix.v01, matrix.v02 },
				{ matrix.v10, matrix.v11, matrix.v12 },
				{ matrix.v20, matrix.v21, matrix.v22 } };
		double[][] vArray = new double[3][3];
		double[] eArray = new double[3];

		svdcmp(uArray, eArray, vArray);

		resultU.v00 = (float) uArray[0][0];
		resultU.v01 = (float) uArray[0][1];
		resultU.v02 = (float) uArray[0][2];
		resultU.v10 = (float) uArray[1][0];
		resultU.v11 = (float) uArray[1][1];
		resultU.v12 = (float) uArray[1][2];
		resultU.v20 = (float) uArray[2][0];
		resultU.v21 = (float) uArray[2][1];
		resultU.v22 = (float) uArray[2][2];

		resultV.v00 = (float) vArray[0][0];
		resultV.v01 = (float) vArray[0][1];
		resultV.v02 = (float) vArray[0][2];
		resultV.v10 = (float) vArray[1][0];
		resultV.v11 = (float) vArray[1][1];
		resultV.v12 = (float) vArray[1][2];
		resultV.v20 = (float) vArray[2][0];
		resultV.v21 = (float) vArray[2][1];
		resultV.v22 = (float) vArray[2][2];

		resultE.x = (float) eArray[0];
		resultE.y = (float) eArray[1];
		resultE.z = (float) eArray[2];
	}

	@Override
	public String toString() {
		return
		//
		v00 + "\t" + v01 + "\t" + v02 + "\n" + //
				v10 + "\t" + v11 + "\t" + v12 + "\n" + //
				v20 + "\t" + v21 + "\t" + v22 + "\n";
	}

	private static void svdcmp(double[][] a, double[] resultW,
			double[][] resultV) {
		// number of rows in A
		int m = a.length;
		// number of columns in A
		int n = a[0].length;

		if (m < n) {
			throw new RuntimeException(
					"Number of rows in A must be greater or equal to number of columns");
		}

		int flag, i, its, j, jj, k, l = 0, nm = 0;
		double anorm, c, f, g, h, s, scale, x, y, z;

		double[] rv1 = new double[n];

		// householder reduction to bidiagonal form
		g = scale = anorm = 0.0;

		for (i = 0; i < n; i++) {
			l = i + 1;
			rv1[i] = scale * g;
			g = s = scale = 0;

			if (i < m) {
				for (k = i; k < m; k++) {
					scale += Math.abs(a[k][i]);
				}

				if (scale != 0.0) {
					for (k = i; k < m; k++) {
						a[k][i] /= scale;
						s += a[k][i] * a[k][i];
					}

					f = a[i][i];
					g = -sign(Math.sqrt(s), f);
					h = f * g - s;
					a[i][i] = f - g;

					if (i != n - 1) {
						for (j = l; j < n; j++) {
							for (s = 0.0, k = i; k < m; k++) {
								s += a[k][i] * a[k][j];
							}

							f = s / h;

							for (k = i; k < m; k++) {
								a[k][j] += f * a[k][i];
							}
						}
					}

					for (k = i; k < m; k++) {
						a[k][i] *= scale;
					}
				}
			}

			resultW[i] = scale * g;
			g = s = scale = 0.0;

			if ((i < m) && (i != n - 1)) {
				for (k = l; k < n; k++) {
					scale += Math.abs(a[i][k]);
				}

				if (scale != 0.0) {
					for (k = l; k < n; k++) {
						a[i][k] /= scale;
						s += a[i][k] * a[i][k];
					}

					f = a[i][l];
					g = -sign(Math.sqrt(s), f);
					h = f * g - s;
					a[i][l] = f - g;

					for (k = l; k < n; k++) {
						rv1[k] = a[i][k] / h;
					}

					if (i != m - 1) {
						for (j = l; j < m; j++) {
							for (s = 0.0, k = l; k < n; k++) {
								s += a[j][k] * a[i][k];
							}
							for (k = l; k < n; k++) {
								a[j][k] += s * rv1[k];
							}
						}
					}

					for (k = l; k < n; k++) {
						a[i][k] *= scale;
					}
				}
			}
			anorm = Math.max(anorm, (Math.abs(resultW[i]) + Math.abs(rv1[i])));
		}

		// accumulation of right-hand transformations
		for (i = n - 1; i >= 0; i--) {
			if (i < n - 1) {
				if (g != 0.0) {
					for (j = l; j < n; j++) {
						resultV[j][i] = (a[i][j] / a[i][l]) / g;
					}

					for (j = l; j < n; j++) {
						for (s = 0, k = l; k < n; k++) {
							s += a[i][k] * resultV[k][j];
						}
						for (k = l; k < n; k++) {
							resultV[k][j] += s * resultV[k][i];
						}
					}
				}
				for (j = l; j < n; j++) {
					resultV[i][j] = resultV[j][i] = 0;
				}
			}
			resultV[i][i] = 1;
			g = rv1[i];
			l = i;
		}

		// accumulation of left-hand transformations
		for (i = n - 1; i >= 0; i--) {
			l = i + 1;
			g = resultW[i];

			if (i < n - 1) {
				for (j = l; j < n; j++) {
					a[i][j] = 0.0;
				}
			}

			if (g != 0) {
				g = 1.0 / g;

				if (i != n - 1) {
					for (j = l; j < n; j++) {
						for (s = 0, k = l; k < m; k++) {
							s += a[k][i] * a[k][j];
						}

						f = (s / a[i][i]) * g;

						for (k = i; k < m; k++) {
							a[k][j] += f * a[k][i];
						}
					}
				}

				for (j = i; j < m; j++) {
					a[j][i] *= g;
				}
			} else {
				for (j = i; j < m; j++) {
					a[j][i] = 0;
				}
			}
			++a[i][i];
		}

		// diagonalization of the bidiagonal form: Loop over singular values
		// and over allowed iterations
		for (k = n - 1; k >= 0; k--) {
			for (its = 1; its <= 30; its++) {
				flag = 1;

				for (l = k; l >= 0; l--) {
					// test for splitting
					nm = l - 1;

					if (Math.abs(rv1[l]) + anorm == anorm) {
						flag = 0;
						break;
					}

					if (Math.abs(resultW[nm]) + anorm == anorm)
						break;
				}

				if (flag != 0) {
					c = 0.0;
					s = 1.0;
					for (i = l; i <= k; i++) {
						f = s * rv1[i];

						if (Math.abs(f) + anorm != anorm) {
							g = resultW[i];
							h = pythag(f, g);
							resultW[i] = h;
							h = 1.0 / h;
							c = g * h;
							s = -f * h;

							for (j = 1; j <= m; j++) {
								y = a[j][nm];
								z = a[j][i];
								a[j][nm] = y * c + z * s;
								a[j][i] = z * c - y * s;
							}
						}
					}
				}

				z = resultW[k];

				if (l == k) {
					// convergence
					if (z < 0.0) {
						// singular value is made nonnegative
						resultW[k] = -z;

						for (j = 0; j < n; j++) {
							resultV[j][k] = -resultV[j][k];
						}
					}
					break;
				}

				if (its == 30) {
					throw new RuntimeException(
							"No convergence in 30 svdcmp iterations");
				}

				// shift from bottom 2-by-2 minor
				x = resultW[l];
				nm = k - 1;
				y = resultW[nm];
				g = rv1[nm];
				h = rv1[k];
				f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
				g = pythag(f, 1.0);
				f = ((x - z) * (x + z) + h * ((y / (f + sign(g, f))) - h)) / x;

				// next QR transformation
				c = s = 1.0;

				for (j = l; j <= nm; j++) {
					i = j + 1;
					g = rv1[i];
					y = resultW[i];
					h = s * g;
					g = c * g;
					z = pythag(f, h);
					rv1[j] = z;
					c = f / z;
					s = h / z;
					f = x * c + g * s;
					g = g * c - x * s;
					h = y * s;
					y *= c;

					for (jj = 0; jj < n; jj++) {
						x = resultV[jj][j];
						z = resultV[jj][i];
						resultV[jj][j] = x * c + z * s;
						resultV[jj][i] = z * c - x * s;
					}

					z = pythag(f, h);
					resultW[j] = z;

					if (z != 0) {
						z = 1.0 / z;
						c = f * z;
						s = h * z;
					}

					f = c * g + s * y;
					x = c * y - s * g;

					for (jj = 0; jj < m; jj++) {
						y = a[jj][j];
						z = a[jj][i];
						a[jj][j] = y * c + z * s;
						a[jj][i] = z * c - y * s;
					}
				}

				rv1[l] = 0.0;
				rv1[k] = f;
				resultW[k] = x;
			}
		}
	}

	private static double sign(double a, double b) {
		return (b >= 0.0) ? Math.abs(a) : -Math.abs(a);
	}

	private static double pythag(double a, double b) {
		double at = Math.abs(a), bt = Math.abs(b), ct, result;

		if (at > bt) {
			ct = bt / at;
			result = at * Math.sqrt(1.0 + ct * ct);
		} else if (bt > 0.0) {
			ct = at / bt;
			result = bt * Math.sqrt(1.0 + ct * ct);
		} else {
			result = 0.0;
		}

		return result;
	}

}
