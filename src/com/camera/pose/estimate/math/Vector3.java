package com.camera.pose.estimate.math;

public class Vector3 {

	public float x, y, z;

	public Vector3(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Vector3(float value) {
		this.x = this.y = this.z = value;
	}

	public Vector3() {
	}

	public void fill(Vector3 vector) {
		this.x = vector.x;
		this.y = vector.y;
		this.z = vector.z;
	}

	public int minIndex() {
		return (x <= y) ? ((x <= z) ? 0 : 2) : ((y <= z) ? 1 : 2);
	}

	public float squaredLength() {
		return x * x + y * y + z * z;
	}

	public float normalize() {
		float norm = (float) Math.sqrt(x * x + y * y + z * z);
		float invNorm = 1.0f / norm;

		x *= invNorm;
		y *= invNorm;
		z *= invNorm;

		return norm;
	}

	public static Vector3 add(Vector3 vector1, Vector3 vector2) {
		return new Vector3(vector1.x + vector2.x, vector1.y + vector2.y,
				vector1.z + vector2.z);
	}

	public static Vector3 add(Vector3 vector, float value) {
		return new Vector3(vector.x + value, vector.y + value, vector.z + value);
	}

	public static Vector3 subtract(Vector3 vector1, Vector3 vector2) {
		return new Vector3(vector1.x - vector2.x, vector1.y - vector2.y,
				vector1.z - vector2.z);
	}

	public static Vector3 subtract(Vector3 vector, float value) {
		return new Vector3(vector.x - value, vector.y - value, vector.z - value);
	}

	public static Vector3 multiply(Vector3 vector1, Vector3 vector2) {
		return new Vector3(vector1.x * vector2.x, vector1.y * vector2.y,
				vector1.z * vector2.z);
	}

	public static Vector3 multiply(Vector3 vector, float factor) {
		return new Vector3(vector.x * factor, vector.y * factor, vector.z
				* factor);
	}

	public static Vector3 divide(Vector3 vector1, Vector3 vector2) {
		return new Vector3(vector1.x / vector2.x, vector1.y / vector2.y,
				vector1.z / vector2.z);
	}

	public static Vector3 divide(Vector3 vector, float factor) {
		return new Vector3(vector.x / factor, vector.y / factor, vector.z
				/ factor);
	}

	public static Vector3 inverse(Vector3 vector) {
		return new Vector3((vector.x == 0) ? 0 : 1.0f / vector.x,
				(vector.y == 0) ? 0 : 1.0f / vector.y, (vector.z == 0) ? 0
						: 1.0f / vector.z);
	}

	public static Vector3 cross(Vector3 vector1, Vector3 vector2) {
		return new Vector3(vector1.y * vector2.z - vector1.z * vector2.y,
				vector1.z * vector2.x - vector1.x * vector2.z, vector1.x
						* vector2.y - vector1.y * vector2.x);
	}

	public static float dot(Vector3 vector1, Vector3 vector2) {
		return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z
				* vector2.z;
	}

	public static final Vector3[] cloneArray(Vector3[] vector3) {
		Vector3[] newVectors = new Vector3[vector3.length];
		for (int i = 0; i < newVectors.length; i++) {
			newVectors[i] = new Vector3(vector3[i].x, vector3[i].y,
					vector3[i].z);
		}
		return newVectors;
	}

	@Override
	public String toString() {
		return "X:" + x + ",Y:" + y + ",Z:" + z;
	}
}
