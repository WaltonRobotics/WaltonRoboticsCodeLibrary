package org.waltonrobotics.controller;

public class VelocityVector {

	private final double leftVelocity;
	private final double rightVelocity;

	public VelocityVector(double leftVelocity, double rightVelocity) {
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
