package com.camera.pose.estimate.algorithm.vpe_old;

public class RealPoint implements Comparable<RealPoint> {
	public final float x;
	public final float y;
	public final float z;

	public RealPoint(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public RealPoint offset(Vector3D vector) {
		return new RealPoint(x + vector.x, y + vector.y, z + vector.z);
	}

	@Override
	public String toString() {
		return "x:" + x + ",y:" + y + ",z:" + z;
	}

	@Override
	public int compareTo(RealPoint that) {
		int result = compare(this.x, that.x);
		if (0 != result)
			return result;

		result = compare(this.y, that.y);
		if (0 != result)
			return result;

		return compare(this.z, that.z);
	}

	private static int compare(double a, double b) {
		return a > b ? 1 : (a < b ? -1 : 0);
	}

}
