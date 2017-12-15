package org.waltonrobotics.controller;

/**
 * Utilized to identify left and right encoder measurements
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class RobotPair {

	private final double left;
	private final double right;

	public RobotPair(double left, double right) {
		this.left = left;
		this.right = right;
		System.out.println(left + "\t" + right);
	}

	public double getLeft() {
		return left;
	}

	public double getRight() {
		return right;
	}

}
