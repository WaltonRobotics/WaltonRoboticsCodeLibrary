package org.waltonrobotics.controller;

/**
 * Holds an encoder length, velocity, and acceleration
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class State {

	private final double length;
	private final double velocity;
	private final double acceleration;

	/**
	 * @param length
	 * @param velocity
	 * @param acceleration
	 */
	public State(double length, double velocity, double acceleration) {
		this.length = length;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}

	/**
	 * 
	 * @return length
	 */
	public double getLength() {
		return length;
	}

	/**
	 * 
	 * @return velocity
	 */
	public double getVelocity() {
		return velocity;
	}

	/**
	 * 
	 * @return acceleration
	 */
	public double getAcceleration() {
		return acceleration;
	}

}
