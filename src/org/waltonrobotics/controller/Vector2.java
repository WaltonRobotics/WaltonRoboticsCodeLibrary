package org.waltonrobotics.controller;

public class Vector2 {
	private final double leftVelocity;
	private final double rightVelocity;

	public Vector2(double leftVelocity, double rightVelocity) {
		this.leftVelocity = leftVelocity;
		this.rightVelocity = rightVelocity;
	}

	public double getLeftVelocity() {
		return leftVelocity;
	}

	public double getRightVelocity() {
		return rightVelocity;
	}

}
