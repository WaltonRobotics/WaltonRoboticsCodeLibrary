package org.waltonrobotics.motion.curve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;
import org.waltonrobotics.motion.Path;

public class Spline implements Path {

	private Point[] knots;
	private Point[] pathPoints;

	public Spline(int numberOfSteps, Point... knots) {
		this.knots = knots;
		computeControlPoints(numberOfSteps);
	}

	private void computeControlPoints(int numberOfSteps)
	{
		int degree = knots.length - 1;
		Point[] points1 = new Point[degree];
		Point[] points2 = new Point[degree];
		
		/*constants for Thomas Algorithm*/
		double[] a = new double[degree + 1];
		double[] b = new double[degree + 1];
		double[] c = new double[degree + 1];
		double[] r_x = new double[degree + 1];
		double[] r_y = new double[degree + 1];
		
		/*left most segment*/
		a[0]=0;
		b[0]=2;
		c[0]=1;
		r_x[0] = knots[0].getX()+2*knots[1].getX();
		r_y[0] = knots[0].getY()+2*knots[1].getY();
		
		/*internal segments*/
		for (int i = 1; i < degree - 1; i++)
		{
			a[i]=1;
			b[i]=4;
			c[i]=1;
			r_x[i] = 4 * knots[i].getX() + 2 * knots[i+1].getX();
			r_y[i] = 4 * knots[i].getY() + 2 * knots[i+1].getY();
		}
				
		/*right segment*/
		a[degree-1]=2;
		b[degree-1]=7;
		c[degree-1]=0;
		r_x[degree-1] = 8*knots[degree-1].getX()+knots[degree].getX();
		r_y[degree-1] = 8*knots[degree-1].getY()+knots[degree].getY();
		
		/*solves Ax=b with the Thomas algorithm*/
		for (int i = 1; i < degree; i++)
		{
			double m = a[i]/b[i-1];			//temporary variable
			b[i] = b[i] - m * c[i - 1];
			r_x[i] = r_x[i] - m*r_x[i-1];
			r_y[i] = r_y[i] - m*r_y[i-1];
		}
		points1[degree-1] = new Point(r_x[degree-1]/b[degree-1], r_y[degree-1]/b[degree-1]);
		for (int i = degree - 2; i >= 0; --i) {
			points1[i] = new Point((r_x[i] - c[i] * points1[i+1].getX()) / b[i],(r_y[i] - c[i] * points1[i+1].getY()) / b[i]);
		}
			
		/*we have p1, now compute p2*/
		for (int i = 0; i < degree-1;i++) {
			points2[i]=new Point(2*knots[i+1].getX()-points1[i+1].getX(),2*knots[i+1].getY()-points1[i+1].getY());
		}
		
		points2[degree-1] = new Point(0.5 * (knots[degree].getX() + points1[degree-1].getX()),0.5 * (knots[degree].getX() + points1[degree-1].getX()));
		
		
		List<Point> pathPoints = new ArrayList<>();
		/*create the bezier curves to stitch together*/
		for(int i = 0; i < degree; i++) {
			BezierCurve curve = new BezierCurve(numberOfSteps, knots[i], points1[i], points2[i], knots[i+1]);
			Collections.addAll(pathPoints, curve.getPathPoints());
		}
		
		this.pathPoints = pathPoints.stream().toArray(Point[]::new);
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

}
