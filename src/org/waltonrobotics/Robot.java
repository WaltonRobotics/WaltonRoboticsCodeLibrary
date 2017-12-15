package org.waltonrobotics;

/**
 * This is an example Robot class. To use our motions for autonomous, you need
 * to extend AbstractRobot and create a RobotConfiguration, as seen in this
 * class. Everything here needs to be here to be able to use the motions
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class Robot extends AbstractRobot {

	/**
	 * This is needed to use our motions
	 */

	public static RobotConfiguration config = new RobotConfiguration() {

		@Override
		public int getNumberOfSteps() {
			return 100;
		}

		@Override
		public double getDistancePerPulse() {
			return 0.0005;
		}

		@Override
		public int getMotionsCalculationPeriod() {
			return 5;
		}

		@Override
		public DriveTrainInterface getDriveTrain() {
			return new SampleDriveTrain();
		}

		@Override
		public double getKScaling() {
			return 0;
		}

		@Override
		public double getKVoltage() {
			return 0.5;
		}

		@Override
		public double getKCurrent() {
			return 0.1;
		}

		@Override
		public double getKPower() {
			return 20;
		}
	};

	@Override
	protected RobotConfiguration initRobotConfiguration() {
		return config;
	}

}
