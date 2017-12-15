package org.waltonrobotics;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.BezierCurve;
import org.waltonrobotics.motion.Spline;

public class TestCurves {

	public static int steps = 100;
	public static double width = 0.25;

	private static Point[] points = new Point[] { new Point(0, 0), new Point(0, 5), new Point(3, 6), new Point(-5, 3),
			new Point(-10, 3) };

	/**
	 * Run this class to test outputs of Spline and Bezier Curve
	 */
	public static void main(String[] args) {
		new Robot();
		System.out.println("Bezier Curve:");
		BezierCurve curve = new BezierCurve(2, 2, 0, 0, steps, width, points);
		Point[] centerPoints = curve.getPathPoints();
		Point[] leftPoints = curve.getLeftPath();
		Point[] rightPoints = curve.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			printValues(leftPoints[i].getX(), leftPoints[i].getY(), rightPoints[i].getX(), rightPoints[i].getY(),
					centerPoints[i].getDerivative(), leftPoints[i].getVelocity(), rightPoints[i].getVelocity(),
					rightPoints[i].getAcceleration());
		}
		System.out.println("Spline:");
		Spline spline = new Spline(2, 2, steps, width, points);
		centerPoints = spline.getPathPoints();
		leftPoints = spline.getLeftPath();
		rightPoints = spline.getRightPath();
		for (int i = 0; i < centerPoints.length; i++) {
			printValues(leftPoints[i].getX(), leftPoints[i].getY(), rightPoints[i].getX(), rightPoints[i].getY(),
					centerPoints[i].getDerivative(), leftPoints[i].getVelocity(), rightPoints[i].getVelocity(),
					leftPoints[i].getAcceleration());
		}
	}

	private static void printValues(double xL, double yL, double xR, double yR, double dydx, double vL, double vR,
			double a) {
		System.out.printf(
				"xL: %01.03f \t yL: %01.03f \t xR: %01.03f \t yR: %01.03f \t dy/dx: %01.03f \t vL: %01.03f \t vR: %01.03f \t a: %01.03f \n",
				xL, yL, xR, yR, dydx, vL, vR, a);
	}
}
