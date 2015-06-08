package com.camera.pose.estimate.common;

import java.util.concurrent.ConcurrentLinkedQueue;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

public class DebugView extends View {

	public DebugView(Context context, AttributeSet attrs, int defStyleAttr) {
		super(context, attrs, defStyleAttr);
		initialzie(context);
	}

	public DebugView(Context context, AttributeSet attrs) {
		super(context, attrs);
		initialzie(context);
	}

	public DebugView(Context context) {
		super(context);
		initialzie(context);
	}

	private final Paint mPaint = new Paint();

	private void initialzie(Context context) {
		mPaint.setColor(Color.RED);
		mPaint.setStrokeWidth(10);
	}

	@Override
	protected void onDraw(Canvas canvas) {
		Point next = null;
		while (null != (next = pointsToDraw.poll())) {
			mPaint.setStrokeWidth(next.size);
			mPaint.setColor(next.color);
			canvas.drawPoint(next.x, next.y, mPaint);
		}
	}

	private ConcurrentLinkedQueue<Point> pointsToDraw = new ConcurrentLinkedQueue<Point>();

	public DebugView point(double x, double y, int color, int size) {
		pointsToDraw.add(new Point(x, y, color, size));
		return this;
	}

	public void showDebug() {
		invalidate();
	}

	private static class Point {
		private final int x;
		private final int y;
		private final int color;
		private final int size;

		public Point(double x, double y, int color, int size) {
			this.x = (int) x;
			this.y = (int) y;
			this.color = color;
			this.size = size;
		}
	}
}