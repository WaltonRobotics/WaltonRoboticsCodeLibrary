package org.waltonrobotics.motion.curve;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;
import org.waltonrobotics.motion.Path;

/**
 * Resources:
 * https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
 */
public class BezierCurve implements Path {

	private final Point[] pathPoints;

	private final double robotLength;
	private final Point[] controlPoints;
	private final Vector2[] vectors;
	private double[] coefficients;

	public BezierCurve(int numberOfSteps, double robotLength, Point... controlPoints) {
		super();
		this.robotLength = robotLength;
		this.controlPoints = controlPoints;

		updateCoefficients();
		pathPoints = getCurvePoints(numberOfSteps);
		vectors = getVectors(numberOfSteps);
	}

	/**
	 * n! / i!(n-i)!
	 */
	private static double findNumberOfCombination(double n, double i) {
		double nFactorial = factorial(n);
		double iFactorial = factorial(i);
		double nMinusIFactorial = factorial(n - i);

		return nFactorial / (iFactorial * nMinusIFactorial);
	}

	/**
	 * for decimal number and integers
	 */
	private static double factorial(double d) {
		double r = d - Math.floor(d) + 1;
		for (; d > 1; d -= 1) {
			r *= d;
		}
		return r;
	}

	private Point[] getCurvePoints(int numberOfSteps) {
		Point[] point2DList = new Point[numberOfSteps + 1];

		for (double i = 0; i <= numberOfSteps; i++) {
			point2DList[(int) i] = getPoint(i / ((double) numberOfSteps));
		}

		return point2DList;
	}

	private Vector2[] getVectors(int numberOfSteps) {
		Vector2[] point2DList = new Vector2[numberOfSteps + 1];

		for (double i = 0; i <= numberOfSteps; i++) {
			point2DList[(int) i] = getVelocity(i / ((double) numberOfSteps));
		}

		return point2DList;
	}

	private void updateCoefficients() {
		int n = getDegree();
		coefficients = new double[n + 1];
		for (int i = 0; i < coefficients.length; i++) {
			coefficients[i] = findNumberOfCombination(n, i);
		}
	}

	private Point getPoint(double percentage) {
		double xCoordinateAtPercentage = 0;
		double yCoordinateAtPercentage = 0;

		int n = getDegree();

		for (double i = 0; i <= n; i++) {
			double coefficient = coefficients[(int) i];

			double oneMinusT = Math.pow(1 - percentage, n - i);

			double powerOfT = Math.pow(percentage, i);

			Point pointI = controlPoints[(int) i];

			xCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getX());
			yCoordinateAtPercentage += (coefficient * oneMinusT * powerOfT * pointI.getY());
		}

		return new Point(xCoordinateAtPercentage, yCoordinateAtPercentage, getDerivative(percentage));
	}

	private Vector2 getVelocity(double percentage) {
		double leftVelocity = 0;
		double rightVelocity = 0;

		int n = getDegree() - 1;

		for (double i = 0; i <= n; i++) {
			double coefficient = findNumberOfCombination(n, i);

			double oneMinusT = Math.pow(1 - percentage, n - i); //TODO Fix math

			double powerOfT = Math.pow(percentage, i);

			double dt = getDerivative(percentage);

			leftVelocity += (coefficient * oneMinusT * powerOfT * (dt));
			rightVelocity += (coefficient * oneMinusT * powerOfT * (dt));
		}

		double slope = rightVelocity / leftVelocity;

		leftVelocity = leftVelocity - (robotLength / 2 * slope);
		rightVelocity = rightVelocity + (robotLength / 2 * slope);

		return new Vector2(leftVelocity, rightVelocity);
	}

	private int getDegree() {
		return controlPoints.length - 1;
	}

	/**
	 * 
	 * @param t
	 *            - percent along curve
	 * @return derivative at point
	 */
	private double getDerivative(double t) {
		int n = getDegree();
		double dx = 0;
		double dy = 0;
		for (int i = 0; i < n; i++) {
			double coefficient = findNumberOfCombination(n - 1, i) * Math.pow(t, i) * Math.pow(t, n - 1 - i);
			dx += coefficient * n * (controlPoints[i + 1].getX() - controlPoints[i].getX());
			dy += coefficient * n * (controlPoints[i + 1].getY() - controlPoints[i].getY());
		}
		return dy / dx;
	}

	@Override
	public Vector2[] getSpeedVectors() {
		return vectors;
	}

	@Override
	public Point[] getPathPoints() {
		return pathPoints;
	}

	@Override
	public Point[] getLeftPath() {
		return new Point[0];
	}

	@Override
	public Point[] getRightPath() {
		return new Point[0];
	}
}
