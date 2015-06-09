package com.camera.pose.estimate.common;

import java.util.concurrent.ConcurrentLinkedQueue;

import android.content.Context;
import android.graphics.Bitmap;
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
	private Bitmap mBitmap;
	private String mText;

	public DebugView setBitmap(Bitmap bmp) {
		this.mBitmap = bmp;
		return this;
	}

	private void initialzie(Context context) {
		mPaint.setColor(Color.RED);
		mPaint.setStrokeWidth(10);
	}

	@Override
	protected void onDraw(Canvas canvas) {
		if (null != mBitmap)
			canvas.drawBitmap(mBitmap, 0, 0, null);

		Point nextPoint = null;
		while (null != (nextPoint = pointsToDraw.poll())) {
			mPaint.setStrokeWidth(nextPoint.size);
			mPaint.setColor(nextPoint.color);
			canvas.drawPoint(nextPoint.x, nextPoint.y, mPaint);
		}

		Point[] nextLine = null;
		while (null != (nextLine = linesToDraw.poll())) {
			mPaint.setStrokeWidth(nextLine[0].size);
			mPaint.setColor(nextLine[0].color);
			canvas.drawLine(nextLine[0].x, nextLine[0].y, nextLine[1].x, nextLine[1].y, mPaint);
		}

		mPaint.setColor(Color.BLACK);
		mPaint.setTextSize(20);
		if (null != mText)
			canvas.drawText(mText, 20, 20, mPaint);
	}

	public DebugView text(String text) {
		this.mText = text;
		return this;
	}

	private ConcurrentLinkedQueue<Point> pointsToDraw = new ConcurrentLinkedQueue<Point>();

	public DebugView point(double x, double y, int color, int size) {
		pointsToDraw.add(new Point(x, y, color, size));
		return this;
	}

	private ConcurrentLinkedQueue<Point[]> linesToDraw = new ConcurrentLinkedQueue<Point[]>();

	public DebugView line(double x1, double y1, double x2, double y2, int color, int width) {
		linesToDraw.add(new Point[] { new Point(x1, y1, color, width), new Point(x2, y2, color, width) });
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