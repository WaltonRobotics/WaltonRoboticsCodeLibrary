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
