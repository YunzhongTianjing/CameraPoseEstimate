package com.camera.pose.estimate.algorithm.coplanar_posit;

public class Point {
	public float x, y;

	public Point(float x, float y) {
		this.x = x;
		this.y = y;
	}

	public float distanceTo(Point anotherPoint) {
		final float dx = x - anotherPoint.x;
		final float dy = y - anotherPoint.y;
		return (float) Math.sqrt(dx * dx + dy * dy);
	}

	@Override
	public String toString() {
		return "X:" + x + ",Y:" + y;
	}
}
