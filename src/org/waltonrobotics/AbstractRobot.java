package org.waltonrobotics;

/**
 * Needs to be extended by the Robot class to use our motions for autonomous
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public abstract class AbstractRobot {

	protected static RobotConfiguration robotConfiguration;
	
	/**
	 * @return the RobotConfigurtion defined by the Robot class
	 */
	public static final RobotConfiguration getRobotConfiguration() {
		return robotConfiguration;
	}

	/**
	 * Initializes a RobotConfiguration
	 * 
	 * @return - the new RobotConfiguration
	 */
	public abstract RobotConfiguration initRobotConfiguration();

	{
		RobotConfiguration config = getRobotConfiguration();

		if (config == null) {
			throw new RuntimeException("Cannot have a null configuration");
		}

		robotConfiguration = config;
	}

}
