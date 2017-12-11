package org.waltonrobotics.controller;

import java.util.Collections;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.Robot;
import org.waltonrobotics.motion.Path;

public class MotionController {

	private class MotionTask extends TimerTask {

		@Override
		public void run() {
			calculateVelocities();
		}

	}

	private BlockingDeque<Path> paths = new LinkedBlockingDeque<Path>();
	private Timer controller;
	private boolean running;
	private int nSteps;
	private int period;

	/**
	 * Creates the class that calculates the voltages to set the wheels to
	 * 
	 * @param nSteps
	 *            - the amount of steps for paths
	 * @param period
	 *            - the time (milliseconds) between each calculation
	 */
	public MotionController(int nSteps, int period) {
		controller = new Timer();
		this.nSteps = nSteps;
		this.period = period;
		enableScheduler();
	}

	/**
	 * Adds a path to the path queue
	 * 
	 * @param paths
	 *            - the paths to add to the queue
	 */
	public void addPaths(Path... paths) {
		for (Path path : paths) {
			this.paths.addLast(path);
		}
	}

	// TODO Fix me, work with Soham, Tim
	private void calculateVelocities() {

		double leftPower = 0;
		double rightPower = 0;
		boolean enabled;

		synchronized (this) {
			enabled = this.running;
		}

		if (enabled) {

			double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
			double kVoltage = Robot.getRobotConfiguration().getKVoltage();
			double kScaling = Robot.getRobotConfiguration().getKScaling();
			double kCurrent = Robot.getRobotConfiguration().getKCurrent();
			double kPower = Robot.getRobotConfiguration().getKPower();
			RobotPair wheelPositions = Robot.getRobotConfiguration().getDriveTrain().getWheelPositions();

			KinematicPose kinematicPose;
			if (currentKinematics != null) {
				kinematicPose = currentKinematics.interpolatePose(time);
			} else {
				kinematicPose = staticKinematicPose;
			}
			synchronized (this) {
				// feed forward
				leftPower += (kVoltage * kinematicPose.left.velocity + kScaling)
						+ kCurrent * kinematicPose.left.acceleration;
				rightPower += (kVoltage * kinematicPose.right.velocity + kScaling)
						+ kCurrent * kinematicPose.right.acceleration;
				// feed back
				leftPower += kPower * (kinematicPose.left.length - wheelPositions.getLeft());
				rightPower += kPower * (kinematicPose.right.length - wheelPositions.getRight());

			}

			leftPower = Math.max(-1, Math.min(1, leftPower));
			rightPower = Math.max(-1, Math.min(1, rightPower));

			// Robot.drivetrain.setSpeeds(leftPower, rightPower);

			if (kinematicPose.isFinished) {
				Path newPath = paths.pollFirst();
				if (newPath != null) {
					currentKinematics = new Kinematics(newPath, currentKinematics.getWheelPositions(),
							currentKinematics.getTime(), 0, 0, nSteps);
					// System.out.println("starting new motion:" + currentKinematics.toString());
				} else {
					staticKinematicPose = Kinematics.staticPose(currentKinematics.getPose(),
							currentKinematics.getWheelPositions(), currentKinematics.getTime());
					currentKinematics = null;
				}
			}
		}
	}

	public void clearMotions() {
		paths.clear();
	}

	public void enableScheduler() {
		if (!running) {
			Path newPath = paths.poll();
			if (newPath != null) {
				controller.scheduleAtFixedRate(new MotionTask(), 0L, (long) period);
				running = true;
			}
		}
	}

	public boolean isFinished() {
		return false; // FIXME do this
	}

	public boolean isRunning() {
		return running;
	}

	public void stopScheduler() {
		controller.cancel();
		running = false;
	}
}
