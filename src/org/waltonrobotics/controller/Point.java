package org.waltonrobotics.controller;

public class Point {
	private final double x;
	private final double y;
	private final double headingAngle;

	public Point(double x, double y, double headingAngle) {
		this.x = x;
		this.y = y;
		this.headingAngle = headingAngle;
	}

	public Point(double x, double y) {
		this(x, y, 0);
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getHeadingAngle() {
		return headingAngle;
	}

}
