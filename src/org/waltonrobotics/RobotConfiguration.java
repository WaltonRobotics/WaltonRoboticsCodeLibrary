package org.waltonrobotics;

/**
 * Used in the Robot class to store important values for motion profiling
 * 
 * @author Marius Juston, Walton Robotics
 * @author Russell Newton, Walton Robotics
 *
 */
public interface RobotConfiguration {
	public double getDistancePerPulse();

	public int getNumberOfSteps();

	public int getMotionsCalculationPeriod();

	public DriveTrainInterface getDriveTrain();

	public double getKScaling();

	public double getKVoltage();

	public double getKCurrent();

	public double getKPower();
}