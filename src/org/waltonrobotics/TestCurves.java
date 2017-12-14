package org.waltonrobotics;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.curve.BezierCurve;
import org.waltonrobotics.motion.curve.Spline;

public class TestCurves {
	
	public static int steps = 100;
	public static double width = 0;

	private static Point a = new Point(0, 2);
	private static Point b = new Point(5, -12);
	private static Point c = new Point(7, 12);
	private static Point d = new Point(-7, 3);
	private static Point e = new Point(2, 0);

	/**
	 * Run this class to test outputs of Spline and Bezier Curve
	 */
	public static void main(String[] args) {
		new Robot();
		System.out.println("Bezier Curve with control points");
		BezierCurve curve = new BezierCurve(1, 1, 1, 1, steps, width, a, b, c, d, e);
		Point[] centerPoints = curve.getPathPoints();
		Point[] leftPoints = curve.getLeftPath();
		Point[] rightPoints = curve.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			locateSidePoints(leftPoints[i].getX(), leftPoints[i].getY(), rightPoints[i].getX(), rightPoints[i].getY(),
					centerPoints[i].getDerivative());
		}
		System.out.println("Spline with knots");
		Spline spline = new Spline(1, 1, steps, width, a, b, c, d, e);
		centerPoints = spline.getPathPoints();
		leftPoints = spline.getLeftPath();
		rightPoints = spline.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			locateSidePoints(leftPoints[i].getX(), leftPoints[i].getY(), rightPoints[i].getX(), rightPoints[i].getY(),
					centerPoints[i].getDerivative());
		}
	}

	private static void locateSidePoints(double xL, double yL, double xR, double yR, double dt) {
		System.out.printf("xL: %01.03f \t yL: %01.03f \t xR: %01.03f \t yR: %01.03f \t dt: %01.03f \n", xL, yL, xR, yR,
				dt);
	}
}
