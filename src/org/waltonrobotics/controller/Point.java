package org.waltonrobotics.controller;

/**
 * Used to define a point in space with an x, y, derivative, velocity, acceleration, and average encoder distance
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
	private final double length;
	private final double lCenter;
	private final double time;

	/**
	 * Used to create a point
	 * 
	 * @param x
	 * @param y
	 * @param derivative
	 * @param velocity
	 * @param acceleration
	 * @param length - desired encoder distance
	 * @param lCenter - average desired encoder distance
	 */
	public Point(double x, double y, double derivative, double velocity, double acceleration, double length, double lCenter, double time) {
		this.x = x;
		this.y = y;
		this.derivative = derivative;
		this.velocity = velocity;
		this.acceleration = acceleration;
		this.length = length;
		this.lCenter = lCenter;
		this.time = time;
	}

	/**
	 * Can be used to create a point with just x, y, and derivative
	 * 
	 * @param x
	 * @param y
	 * @param derivative
	 */
	public Point(double x, double y, double derivative) {
		this(x, y, derivative, 0, 0, 0, 0, 0);
	}
	
	/**
	 * Can be used to create a point with just x and y
	 * 
	 * @param x
	 * @param y
	 */
	public Point(double x, double y) {
		this(x, y, 0, 0, 0, 0, 0, 0);
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
	
	public double getLength() {
		return length;
	}
	
	public double getLCenter() {
		return lCenter;
	}
	
	public double getTime() {
		return time;
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
	 * @param length
	 * @param lCenter
	 * @param time
	 * @return the offset point
	 */
	public Point offsetPerpendicular(double dtAtPoint, double distance, double velocity, double acceleration, double length, double lCenter, double time) {
		double angleOfDT = Math.atan(dtAtPoint);
		double offsetX = distance * Math.cos(angleOfDT + Math.PI / 2); // Finds point at distance along perpendicular
																		// line
		double offsetY = distance * Math.sin(angleOfDT + Math.PI / 2);

		return new Point(this.x + offsetX, this.y + offsetY, angleOfDT, velocity, acceleration, length, lCenter, time);
	}
	
	public double distance(Point previousPoint) {
		return Math.sqrt(Math.pow(this.x - previousPoint.getX(), 2) + Math.pow(this.y - previousPoint.getY(), 2));
	}
	
	public Point interpolate(Point previousPoint, double time) {
		
		return null;
	}

}
