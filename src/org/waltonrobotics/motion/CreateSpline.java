package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.curve.Spline;

public class CreateSpline {

	public static int steps;
	public static double robotWidth;

	private Spline spline;

	public CreateSpline(double vCruise, double aMax, int steps, double robotWidth, Point... knots) {
		this.steps = steps;
		this.robotWidth = robotWidth;
		spline = new Spline(vCruise, aMax, steps, robotWidth, knots);
	}

}
