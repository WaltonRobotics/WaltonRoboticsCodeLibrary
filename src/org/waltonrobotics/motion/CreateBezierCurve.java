package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.curve.BezierCurve;

public class CreateBezierCurve {

	public static int steps;
	public static double robotWidth;

	private BezierCurve bezierCurve;

	public CreateBezierCurve(double vCruise, double aMax, int steps, double robotWidth, Point... controlPoints) {
		this.steps = steps;
		this.robotWidth = robotWidth;
		bezierCurve = new BezierCurve(vCruise, aMax, steps, robotWidth, controlPoints);
	}
}
