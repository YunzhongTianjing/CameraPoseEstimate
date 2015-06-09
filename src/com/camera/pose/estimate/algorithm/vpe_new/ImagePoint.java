package com.camera.pose.estimate.algorithm.vpe_new;

public final class ImagePoint {
	public final float x;
	public final float y;

	public ImagePoint(float x, float y) {
		this.x = x;
		this.y = y;
	}

	public ImagePoint originOfSymmetry() {
		return new ImagePoint(-x, -y);
	}

	@Override
	public String toString() {
		return "x:" + x + ",y:" + y;
	}
}
