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

	/**
	 * Used to create a point
	 * 
	 * @param x
	 * @param y
	 * @param derivative
	 */
	public Point(double x, double y, double derivative) {
		this.x = x;
		this.y = y;
		this.derivative = derivative;
	}

	/**
	 * Can be used to create a point without specifying a derivative
	 * 
	 * @param x
	 * @param y
	 */
	public Point(double x, double y) {
		this(x, y, 0);
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
	 * Offsets a point along a perpendicular line from a tangent line
	 * 
	 * @param dtAtPoint
	 *            - the derivative of the point
	 * @param distance
	 *            - the distance to offset the point by
	 * @return the offset point
	 */
	public Point offsetPerpendicular(double dtAtPoint, double distance) {
		double angleOfDT = Math.atan(dtAtPoint);
		double offsetX = distance * Math.cos(angleOfDT + Math.PI / 2); // Finds point at distance along perpendicular
																		// line
		double offsetY = distance * Math.sin(angleOfDT + Math.PI / 2);

		return new Point(this.x + offsetX, this.y + offsetY, angleOfDT);
	}

}
