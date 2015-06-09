package com.camera.pose.estimate.algorithm.vpe_old;

public class Vector3D {
	public final float x, y, z;

	public Vector3D(RealPoint point1, RealPoint point2) {
		this.x = point2.x - point1.x;
		this.y = point2.y - point1.y;
		this.z = point2.z - point1.z;
	}

	public Vector3D(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Vector3D cross(Vector3D vectorAnother) {
		return new Vector3D(y * vectorAnother.z - vectorAnother.y * z,//
				-(x * vectorAnother.z - vectorAnother.x * z), //
				x * vectorAnother.y - vectorAnother.x * y);
	}

	public float dot(Vector3D vectorAnothor) {
		return x * vectorAnothor.x + y * vectorAnothor.y + z * vectorAnothor.z;
	}

	public float length() {
		return (float) Math.sqrt(x * x + y * y + z * z);
	}

	public Vector3D normalize() {
		final float len = length();
		return new Vector3D(x / len, y / len, z / len);
	}

	public static Vector3D average(Vector3D... vector3ds) {
		if (null == vector3ds || 0 == vector3ds.length)
			return null;
		float x = 0, y = 0, z = 0;
		for (Vector3D vector3d : vector3ds) {
			x += vector3d.x;
			y += vector3d.y;
			z += vector3d.z;
		}
		return new Vector3D(x / vector3ds.length, y / vector3ds.length, z
				/ vector3ds.length);
	}

	@Override
	public String toString() {
		return "x:" + x + ",y:" + y + ",z:" + z;
	}
}
