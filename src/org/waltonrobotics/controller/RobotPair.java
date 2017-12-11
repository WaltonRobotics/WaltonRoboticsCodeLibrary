package org.waltonrobotics.controller;

public class RobotPair {
	
	private final double left;
	private final double right;

	public RobotPair(double left, double right) {
		this.left = left;
		this.right = right;
	}

	public double getLeft() {
		return left;
	}

	public double getRight() {
		return right;
	}
	
}
