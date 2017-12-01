package org.waltonrobotics;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.curve.BezierCurve;
import org.waltonrobotics.motion.curve.Spline;

public class TestCurves {

	public static int steps = 100;
	public static double width = 0.5;

	private static Point a = new Point(0, 0);
	private static Point b = new Point(1, 1);
	private static Point c = new Point(2, 2);
	private static Point d = new Point(3, 3);
	private static Point e = new Point(4, 5);

	/**
	 * Run this class to test outputs of Spline and Bezier Curve
	 */
	public static void main(String[] args) {
		System.out.println("Bezier Curve with control points");
		BezierCurve curve = new BezierCurve(steps, width, a, b, c, d, e);
		Point[] centerPoints = curve.getPathPoints();
		Point[] leftPoints = curve.getLeftPath();
		Point[] rightPoints = curve.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			locateSidePoints(leftPoints[i].getX(), leftPoints[i].getY(), leftPoints[i].getDerivative(),
					rightPoints[i].getX(), rightPoints[i].getY(), rightPoints[i].getDerivative());
		}
		System.out.println("Spline with knots");
		Spline spline = new Spline(steps, width, b, c, d, e);
		centerPoints = spline.getPathPoints();
		leftPoints = spline.getLeftPath();
		rightPoints = spline.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			locateSidePoints(leftPoints[i].getX(), leftPoints[i].getY(), leftPoints[i].getDerivative(),
					rightPoints[i].getX(), rightPoints[i].getY(), rightPoints[i].getDerivative());
		}
	}

	private static void locateSidePoints(double xL, double yL, double dtL, double xR, double yR, double dtR) {
		System.out.printf("xL: %01.03f \t yL: %01.03f \t dtL: %01.03f \t xR: %01.03f \t yR: %01.03f \t dtR: %01.03f \n",
				xL, yL, dtL, xR, yR, dtR);
	}

	private static void locatePoints(double x, double y) {
		System.out.printf("x: %01.03f \t y: %01.03f \n", x, y);
	}
}
