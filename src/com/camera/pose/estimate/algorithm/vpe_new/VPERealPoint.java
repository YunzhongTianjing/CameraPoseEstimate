package com.camera.pose.estimate.algorithm.vpe_new;

import java.text.DecimalFormat;
import java.util.Collections;
import java.util.List;

public class VPERealPoint implements Comparable<VPERealPoint> {
	public final double x;
	public final double y;
	public final double z;

	public VPERealPoint(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	private final static DecimalFormat FORMAT = new DecimalFormat("000.00");

	@Override
	public String toString() {
		return "x:" + FORMAT.format(x) + ",y:" + FORMAT.format(y) + ",z:"
				+ FORMAT.format(z);
	}

	@Override
	public int compareTo(VPERealPoint that) {
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

	public static VPERealPoint getMidiant(List<VPERealPoint> points) {
		Collections.sort(points);
		return points.get(points.size() / 2);
	}
}
