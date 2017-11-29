package org.waltonrobotics;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.curve.BezierCurve;
import org.waltonrobotics.motion.curve.Spline;

public class TestCurves {
	
	private static int steps = 10;	
	private static Point a = new Point(0,0);
	private static Point b = new Point(100,250);
	private static Point c = new Point(250,100);
	private static Point d = new Point(250,0);
	
	/**
	 * Run this class to test outputs of Spline and Bezier Curve
	 */
	public static void main(String[] args) {
		System.out.println("Bezier Curve with control points a, b, c, and d");
		BezierCurve curve = new BezierCurve(steps, a, b, c, d);
		for(Point point : curve.getPathPoints()) {
			locatePoints(point.getX(), point.getY());
		}
		System.out.println("Spline with knots a, b, c, and d");
		Spline spline = new Spline(steps, a, b, c, d);
		for(Point point : spline.getPathPoints()) {
			locatePoints(point.getX(), point.getY());
		}
	}
	
	public static void locatePoints(double x, double y) {
		System.out.printf("x: %01.03f \t y: %01.03f \n", x, y);
	}
}
