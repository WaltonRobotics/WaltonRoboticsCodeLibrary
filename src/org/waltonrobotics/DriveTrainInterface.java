package org.waltonrobotics;

import org.waltonrobotics.controller.RobotPair;
import org.waltonrobotics.motion.Path;

/**
 * Needs to be implemented by the DriveTrain subsystem to use our motions for autonomous
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public interface DriveTrainInterface {
	/**
	 * @return A RobotPair that is the wheel positions
	 */
	public RobotPair getWheelPositions();

	/**
	 * Resets the encoders
	 */
	public void reset();

	/**
	 * @return whether or not the MotionController is running
	 */
	public boolean getControllerStatus();

	/**
	 * Starts the MotionController
	 */
	public void startControllerMotion();

	/**
	 * Cancels the MotionController
	 */
	public void cancelControllerMotion();

	/**
	 * @param paths - paths to add to the MotionController queue
	 */
	public void addControllerMotions(Path... paths);

	/**
	 * @return whether or not the MotionController's queue is empty
	 */
	public boolean isControllerFinished();
}
