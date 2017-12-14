package org.waltonrobotics.controller;

/**
 * Used to define a point in space with an x, y, and a derivative
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class Point {

	private final double x;
	private final double y;
	private final double derivative;
	private final double velocity;
	private final double acceleration;
	private final double lMidpoint;

	/**
	 * Used to create a point
	 * 
	 * @param x
	 * @param y
	 * @param derivative
	 * @param velocity - velocity to get to point from last
	 * @param acceleration - acceleration to get to point from last
	 * @param lMidpoint - desired average encoder distance to get to that point
	 */
	public Point(double x, double y, double derivative, double velocity, double acceleration, double lMidpoint) {
		this.x = x;
		this.y = y;
		this.derivative = derivative;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.lMidpoint = lMidpoint;
	}

	/**
	 * Can be used to create a point without specifying a derivative
	 * 
	 * @param x
	 * @param y
	 */
	public Point(double x, double y) {
		this(x, y, 0, 0, 0, 0);
	}

	/**
	 * @return the x value of the point
	 */
	public double getX() {
		return x;
	}

	/**
	 * @return the y value of the point
	 */
	public double getY() {
		return y;
	}

	/**
	 * @return the derivative of the point
	 */
	public double getDerivative() {
		return derivative;
	}
	/**
	 * @return the velocity at the point
	 */
	public double getVelocity() {
		return velocity;
	}
	
	/**
	 * @return the acceleration at the point
	 */
	public double getAcceleration() {
		return acceleration;
	}
	
	public double getLMidpoint() {
		return lMidpoint;
	}

	/**
	 * Offsets a point along a perpendicular line from a tangent line
	 * 
	 * @param dtAtPoint
	 *            - the derivative of the point
	 * @param distance
	 *            - the distance to offset the point by
	 * @param velocity
	 * @param acceleration
	 * @return the offset point
	 */
	public Point offsetPerpendicular(double dtAtPoint, double distance, double velocity, double acceleration, double lMidpoint) {
		double angleOfDT = Math.atan(dtAtPoint);
		double offsetX = distance * Math.cos(angleOfDT + Math.PI / 2); // Finds point at distance along perpendicular
																		// line
		double offsetY = distance * Math.sin(angleOfDT + Math.PI / 2);

		return new Point(this.x + offsetX, this.y + offsetY, angleOfDT, velocity, acceleration, lMidpoint);
	}
	
	public double distance(Point nextPoint) {
		return Math.sqrt(Math.pow(this.x - nextPoint.getX(), 2) + Math.pow(this.y - nextPoint.getY(), 2));
	}

}
