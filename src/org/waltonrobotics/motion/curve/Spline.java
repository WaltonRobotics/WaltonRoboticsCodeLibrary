package org.waltonrobotics.motion.curve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;
import org.waltonrobotics.motion.Path;

/**
 * Creates splines that travel through set points, or "knots", and all
 * information needed to make the robot travel along the path.
 * 
 * @author Russell Newton, Walton Robotics
 *
 */

public class Spline implements Path {

	public double robotWidth;

	private List<List<Point>> pathControlPoints;
	private Point[] pathPoints;
	private Point[] leftPoints;
	private Point[] rightPoints;
	private int numberOfSteps;

	/**
	 * Create a new spline
	 * 
	 * @param numberOfSteps
	 *            - the amount of points generated for the path, the resolution of the spline
	 * @param knots
	 *            - the fixed points the spline will travel through
	 */
	public Spline(int numberOfSteps, double robotWidth, Point... knots) {
		this.numberOfSteps = numberOfSteps;
		this.robotWidth = robotWidth;
		pathControlPoints = computeControlPoints(knots);
		joinBezierCurves(pathControlPoints);
	}

	/**
	 * Creates the control points required to make cubic bezier curves that
	 * transition between knots.
	 * 
	 * @see https://www.particleincell.com/2012/bezier-splines/
	 * @see https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
	 * 
	 * @param knots
	 * @return A list of lists that hold the control points for the segments in the
	 *         spline
	 */
	private List<List<Point>> computeControlPoints(Point[] knots) {
		int degree = knots.length - 1;
		Point[] points1 = new Point[degree];
		Point[] points2 = new Point[degree];

		/* constants for Thomas Algorithm */
		double[] a = new double[degree];
		double[] b = new double[degree];
		double[] c = new double[degree];
		double[] r_x = new double[degree];
		double[] r_y = new double[degree];

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
				0.5 * (knots[degree].getY() + points1[degree - 1].getY()));
		List<List<Point>> controlPoints = new ArrayList<>();
		for (int i = 0; i < degree; i++) {
			List<Point> segmentControlPoints = new ArrayList<>();
			Collections.addAll(segmentControlPoints, knots[i], points1[i], points2[i], knots[i + 1]);
			Collections.addAll(controlPoints, segmentControlPoints);
		}

		return controlPoints;
	}

	/**
	 * Joins the bezier curves defining the spline into intdividual arrays of Points
	 * 
	 * @param pathControlPoints - a List of Lists of control points for each curve that make up the spline
	 */
	private void joinBezierCurves(List<List<Point>> pathControlPoints) {
		List<Point> pathPointsAdd = new ArrayList<>();
		List<Point> leftPointsAdd = new ArrayList<>();
		List<Point> rightPointsAdd = new ArrayList<>();
		
		for (List<Point> curveControlPoints : pathControlPoints) {
			Point[] controlPoints = curveControlPoints.stream().toArray(Point[]::new);
			BezierCurve curve = new BezierCurve(numberOfSteps, robotWidth, controlPoints);
			
			Point[] pathPoints = curve.getPathPoints();
			Point[] leftPoints = curve.getLeftPath();
			Point[] rightPoints = curve.getRightPath();
			Collections.addAll(pathPointsAdd, pathPoints);
			Collections.addAll(leftPointsAdd, leftPoints);
			Collections.addAll(rightPointsAdd, rightPoints);
		}
		this.pathPoints = pathPointsAdd.stream().toArray(Point[]::new);
		this.leftPoints = leftPointsAdd.stream().toArray(Point[]::new);
		this.rightPoints = rightPointsAdd.stream().toArray(Point[]::new);
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

}
