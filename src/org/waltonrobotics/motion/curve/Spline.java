package org.waltonrobotics.motion.curve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;
import org.waltonrobotics.motion.Path;

public class Spline implements Path {

	public double width = 1;

	private Point[] knots;
	private Point[] pathPoints;
	private Point[] leftPoints;
	private Point[] rightPoints;
	private double[] dts;
	private int numberOfSteps;

	public Spline(int numberOfSteps, Point... knots) {
		this.knots = knots;
		this.numberOfSteps = numberOfSteps;
		dts = new double[numberOfSteps * (knots.length-1)];
		computeControlPoints();
	}

	private void computeControlPoints() {
		int degree = knots.length - 1;
		Point[] points1 = new Point[degree];
		Point[] points2 = new Point[degree];

		/* constants for Thomas Algorithm */
		double[] a = new double[degree + 1];
		double[] b = new double[degree + 1];
		double[] c = new double[degree + 1];
		double[] r_x = new double[degree + 1];
		double[] r_y = new double[degree + 1];

		/* left most segment */
		a[0] = 0;
		b[0] = 2;
		c[0] = 1;
		r_x[0] = knots[0].getX() + 2 * knots[1].getX();
		r_y[0] = knots[0].getY() + 2 * knots[1].getY();

		/* internal segments */
		for (int i = 1; i < degree - 1; i++) {
			a[i] = 1;
			b[i] = 4;
			c[i] = 1;
			r_x[i] = 4 * knots[i].getX() + 2 * knots[i + 1].getX();
			r_y[i] = 4 * knots[i].getY() + 2 * knots[i + 1].getY();
		}

		/* right segment */
		a[degree - 1] = 2;
		b[degree - 1] = 7;
		c[degree - 1] = 0;
		r_x[degree - 1] = 8 * knots[degree - 1].getX() + knots[degree].getX();
		r_y[degree - 1] = 8 * knots[degree - 1].getY() + knots[degree].getY();

		/* solves Ax=b with the Thomas algorithm */
		for (int i = 1; i < degree; i++) {
			double m = a[i] / b[i - 1]; // temporary variable
			b[i] = b[i] - m * c[i - 1];
			r_x[i] = r_x[i] - m * r_x[i - 1];
			r_y[i] = r_y[i] - m * r_y[i - 1];
		}
		points1[degree - 1] = new Point(r_x[degree - 1] / b[degree - 1], r_y[degree - 1] / b[degree - 1]);
		for (int i = degree - 2; i >= 0; --i) {
			points1[i] = new Point((r_x[i] - c[i] * points1[i + 1].getX()) / b[i],
					(r_y[i] - c[i] * points1[i + 1].getY()) / b[i]);
		}

		/* we have p1, now compute p2 */
		for (int i = 0; i < degree - 1; i++) {
			points2[i] = new Point(2 * knots[i + 1].getX() - points1[i + 1].getX(),
					2 * knots[i + 1].getY() - points1[i + 1].getY());
		}

		points2[degree - 1] = new Point(0.5 * (knots[degree].getX() + points1[degree - 1].getX()),
				0.5 * (knots[degree].getX() + points1[degree - 1].getX()));

		List<Point> pathPoints = new ArrayList<>();
		List<Point> leftPoints = new ArrayList<>();
		List<Point> rightPoints = new ArrayList<>();
		/* create the bezier curves to stitch together */
		for (int i = 0; i < degree; i++) {
			BezierCurve curve = new BezierCurve(numberOfSteps, knots[i], points1[i], points2[i], knots[i + 1]);
			Collections.addAll(pathPoints, curve.getPathPoints());
			Point[] leftCurve = offsetPoints(false, curve.getPathPoints(), knots[i], points1[i], points2[i],
					knots[i + 1]);
			Point[] rightCurve = offsetPoints(true, curve.getPathPoints(), knots[i], points1[i], points2[i],
					knots[i + 1]);
			Collections.addAll(leftPoints, leftCurve);
			Collections.addAll(rightPoints, rightCurve);
		}

		this.pathPoints = pathPoints.stream().toArray(Point[]::new);
		this.leftPoints = leftPoints.stream().toArray(Point[]::new);
		this.rightPoints = rightPoints.stream().toArray(Point[]::new);
	}

	private Point[] offsetPoints(boolean isRight, Point[] points, Point CP0, Point CP1, Point CP2, Point CP3) {
		Point[] offsetPoints = new Point[numberOfSteps];
		for (int i = 0; i < numberOfSteps; i++) {
			double dt = getDT(i + 1, CP0, CP1, CP2, CP3);
			if (isRight) {
				offsetPoints[i] = points[i].offsetPerpendicular(dt, width);
			} else {
				offsetPoints[i] = points[i].offsetPerpendicular(dt, -width);
			}
		}
		return offsetPoints;
	}

	private double getDT(int t, Point CP0, Point CP1, Point CP2, Point CP3) {
		double r = 1 - t;
		/* Uses the derivative of the cubic formula to find dx and dy, then dt */
		double dx = CP0.getX() * (-3 * r * r) + CP1.getX() * ((3 * r * r) - (6 * r * t))
				+ CP2.getX() * ((-3 * t * t) + (6 * r * t)) + CP3.getX() * (3 * t * t);
		double dy = CP0.getY() * (-3 * r * r) + CP1.getY() * ((3 * r * r) - (6 * r * t))
				+ CP2.getY() * ((-3 * t * t) + (6 * r * t)) + CP3.getY() * (3 * t * t);
		double dt = dy / dx;
		return dt;
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
		return leftPoints;
	}

	@Override
	public Point[] getRightPath() {
		return rightPoints;
	}

	@Override
	public double[] getDTsOnPath() {
		return dts;
	}

}
