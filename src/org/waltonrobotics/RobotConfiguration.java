package org.waltonrobotics;

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