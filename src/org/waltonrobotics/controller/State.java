package org.waltonrobotics.controller;

public class State {

	private final double length;
	private final double velocity;
	private final double acceleration;

	public State(double length, double velocity, double acceleration) {
		this.length = length;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}

	public double getLength() {
		return length;
	}

	public double getVelocity() {
		return velocity;
	}

	public double getAcceleration() {
		return acceleration;
	}

}
