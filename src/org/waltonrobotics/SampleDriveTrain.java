package org.waltonrobotics;

import org.waltonrobotics.controller.MotionController;
import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.RobotPair;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

/**
 * This is a sample drive train. If you want to use our motions for autonomous,
 * everything here will need to included in your drivetrain subsystem. This
 * includes the left and right encoders, the MotionController, and every
 * inherited method from the DriveTrainInterface
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class SampleDriveTrain implements DriveTrainInterface {

	// private Encoder rightEncoder = RobotMap.rightEncoder;
	// private Encoder leftEncoder = RobotMap.leftEncoder;
	// private CANTalon rightMotor = RobotMap.rightMotor;
	// private CANTalon leftMotor = RobotMap.leftMotor;

	private MotionController controller;

	public SampleDriveTrain() {
		controller = new MotionController(Robot.getRobotConfiguration().getMotionsCalculationPeriod());
		// leftEncoder.setDistancePerPulse(Robot.getRobotConfiguration().getDistancePerPulse());
		// rightEncoder.setDistancePerPulse(-Robot.getRobotConfiguration().getDistancePerPulse());
	}

	@Override
	public RobotPair getWheelPositions() {
		// return new RobotPair(leftEncoder.getDistance(), rightEncoder.getDistance());
		return new RobotPair(0, 0);
	}

	@Override
	public void reset() {
		// leftEncoder.reset();
		// rightEncoder.reset();
	}

	@Override
	public boolean getControllerStatus() {
		return controller.isRunning();
	}

	@Override
	public void startControllerMotion() {
		controller.enableScheduler();

	}

	@Override
	public void cancelControllerMotion() {
		controller.stopScheduler();
	}

	@Override
	public void addControllerMotions(Path... paths) {
		controller.addPaths(paths);
	}

	@Override
	public boolean isControllerFinished() {
		return controller.isFinished();
	}

	@Override
	public void setSpeeds(double leftSpeed, double rightSpeed) {
		// leftMotor.setSpeed(leftSpeed);
		// rightMotor.setSpeed(rightSpeed);
	}
}
