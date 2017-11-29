package org.waltonrobotics.motion.curve;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;
import org.waltonrobotics.motion.Path;

public class BezierCurve implements Path {
	private Point[] pathPoints;
	private Point[] leftPathPoints;
	private Point[] rightPathPoints;

	private Point[] controlPoints;
	private double[] coefficients;

	public BezierCurve(int numberOfSteps, Point... controlPoints) {
		super();
		this.controlPoints = controlPoints;

		updateCoefficients();
		pathPoints = getCurvePoints(numberOfSteps);
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

	public Point[] getCurvePoints(int numberOfSteps) {
		Point[] point2DList = new Point[numberOfSteps + 1];

		for (double i = 0; i <= numberOfSteps; i++) {
			point2DList[(int) i] = getPoint(i / ((double) numberOfSteps));
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

		return new Point(xCoordinateAtPercentage, yCoordinateAtPercentage);
	}

	private int getDegree() {
		return controlPoints.length - 1;
	}

	@Override
	public Vector2[] getSpeedVectors() {
		// TODO Do the math stuff
		return null;
	}

	@Override
	public Point[] getPathPoints() {
		return pathPoints;
	}

	@Override
	public Point[] getLeftPath() {
		return null;
	}

	@Override
	public Point[] getRightPath() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double[] getDTsOnPath() {
		// TODO Auto-generated method stub
		return null;
	}

}
