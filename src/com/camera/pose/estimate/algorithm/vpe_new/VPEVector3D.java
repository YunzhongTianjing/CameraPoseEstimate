package com.camera.pose.estimate.algorithm.vpe_new;

public class VPEVector3D {
	public final double x, y, z;

	public VPEVector3D(VPERealPoint point1, VPERealPoint point2) {
		this.x = point2.x - point1.x;
		this.y = point2.y - point1.y;
		this.z = point2.z - point1.z;
	}

	public VPEVector3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public VPEVector3D cross(VPEVector3D vectorAnother) {
		return new VPEVector3D(y * vectorAnother.z - vectorAnother.y * z,//
				-(x * vectorAnother.z - vectorAnother.x * z), //
				x * vectorAnother.y - vectorAnother.x * y);
	}

	public double dot(VPEVector3D vectorAnothor) {
		return x * vectorAnothor.x + y * vectorAnothor.y + z * vectorAnothor.z;
	}

	public double length() {
		return (double) Math.sqrt(x * x + y * y + z * z);
	}

	public VPEVector3D normalize() {
		final double len = length();
		return new VPEVector3D(x / len, y / len, z / len);
	}

	@Override
	public String toString() {
		return "x:" + x + ",y:" + y + ",z:" + z;
	}
}
