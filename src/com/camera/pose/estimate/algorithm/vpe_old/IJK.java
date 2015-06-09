package com.camera.pose.estimate.algorithm.vpe_old;

public class IJK {
	public final int i;
	public final int j;
	public final int k;

	public IJK(int i, int j, int k) {
		this.i = i;
		this.j = j;
		this.k = k;
	}

	public static IJK[] generateSet() {
		final IJK[] set = new IJK[12];
		set[0] = new IJK(0, 1, 2);
		set[1] = new IJK(0, 1, 3);
		set[2] = new IJK(0, 2, 3);
		set[3] = new IJK(1, 0, 2);
		set[4] = new IJK(1, 0, 3);
		set[5] = new IJK(1, 2, 3);
		set[6] = new IJK(2, 0, 1);
		set[7] = new IJK(2, 0, 3);
		set[8] = new IJK(2, 1, 3);
		set[9] = new IJK(3, 0, 1);
		set[10] = new IJK(3, 0, 2);
		set[11] = new IJK(3, 1, 2);
		return set;
	}

	@Override
	public String toString() {
		return (i + 1) + "" + (j + 1) + "" + (k + 1);
	}

}
