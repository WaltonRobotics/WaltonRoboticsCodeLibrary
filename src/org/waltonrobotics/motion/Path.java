package org.waltonrobotics.motion;

import org.usfirst.frc2974.Testbed.controllers.MotionProvider.LimitMode;
import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.VelocityVector;

/**
 * Defines the methods used by a motion path
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class Path {

	/**
	 * Used in the math to determine how to move the robot
	 */
	public enum LimitMode {
		LimitLinearAcceleration, LimitRotationalAcceleration
	}
	
	protected double vCruise;
	protected double aMax;

	/**
	 * @param vCruise - cruise velocity
	 * @param aMax - max acceleration
	 */
	protected Path(double vCruise, double aMax) {
		if(vCruise == 0)
			throw new IllegalArgumentException("vCruise cannot be 0");
		this.vCruise = vCruise;
		
		if(aMax == 0)
			throw new IllegalArgumentException("aMax cannot be 0");
		this.aMax = aMax;
	}
	
	/**
	 * Finds the points that define the path the robot follows
	 * 
	 * @return an array of points that holds the points along the path
	 */
	public abstract Point[] getPathPoints();

	/**
	 * Finds the points that define the path the left side of the robot follows
	 * 
	 * @return an array of points that holds the points along the path
	 */
	public abstract Point[] getLeftPath();

	/**
	 * Finds the points that define the path the right side of the robot follows
	 * 
	 * @return an array of points that holds the points along the path
	 */
	public abstract Point[] getRightPath();
	
	/** 
	 * @return the LimitMode of the path
	 */
	public abstract LimitMode getLimitMode();
}
