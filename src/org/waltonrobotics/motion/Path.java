package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;

/**
 * Defines the methods used by a motion path
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public interface Path {

	/**
	 * Finds the left and right speeds to set the wheels at any point
	 * 
	 * @return a Vector2 array with left and right speeds
	 */
	public Vector2[] getSpeedVectors();

	/**
	 * Finds the points that define the path the robot follows
	 * 
	 * @return an array of points that holds the points along the path
	 */
	public Point[] getPathPoints();

	/**
	 * Finds the points that define the path the left side of the robot follows
	 * 
	 * @return an array of points that holds the points along the path
	 */
	public Point[] getLeftPath();

	/**
	 * Finds the points that define the path the right side of the robot follows
	 * 
	 * @return an array of points that holds the points along the path
	 */
	public Point[] getRightPath();
}
