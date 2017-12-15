package org.waltonrobotics.controller;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.Robot;

public class MotionController {

	private class MotionTask extends TimerTask {

		@Override
		public void run() {
			if (currentPath != null) {
				calculateSpeeds();
			}
		}

	}

	private BlockingDeque<Path> paths = new LinkedBlockingDeque<Path>();
	private Timer controller;
	private boolean running;
	private int currentStep = 0;
	private int period;
	private Path currentPath = null;
	private double[][] staticState;
	private final double startTime;
	private final int nSteps;

	/**
	 * Creates the class that calculates the voltages to set the wheels to
	 * 
	 * @param nSteps
	 *            - the amount of steps for paths
	 * @param period
	 *            - the time (milliseconds) between each calculation
	 */
	public MotionController(int period) {
		controller = new Timer();
		this.period = period;
		enableScheduler();

		staticState = new double[][] { { 0, 0, 0 }, { 0, 0, 0 } };
//		startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		startTime = 0;
		nSteps = Robot.getRobotConfiguration().getNumberOfSteps();
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

	/**
	 * Calculates the powers to send to the wheels
	 */
	private void calculateSpeeds() {

		double leftPower = 0;
		double rightPower = 0;
		boolean enabled;

		synchronized (this) {
			enabled = this.running;
		}

		if (enabled) {

			double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;

			double kVoltage = Robot.getRobotConfiguration().getKVoltage();
			double kScaling = Robot.getRobotConfiguration().getKScaling();
			double kCurrent = Robot.getRobotConfiguration().getKCurrent();
			double kPower = Robot.getRobotConfiguration().getKPower();

			RobotPair wheelPositions = Robot.getRobotConfiguration().getDriveTrain().getWheelPositions();

			double[][] currentState;
			if (currentPath != null) {
				currentState = interpolatePosition(time);
			} else {
				currentState = staticState;
			}
			try {
				if (currentState == null) {
					currentStep++;
					currentState = interpolatePosition(time);
				}
			} catch (ArrayIndexOutOfBoundsException e) {
				currentPath = paths.pollFirst();
				return;
			}

			synchronized (this) {
				// feed forward
				leftPower += (kVoltage * currentState[0][1] + kScaling) + kCurrent * currentState[0][2];
				rightPower += (kVoltage * currentState[1][1] + kScaling) + kCurrent * currentState[1][2];
				// feed back
				leftPower += kPower * (currentState[0][0] - wheelPositions.getLeft());
				rightPower += kPower * (currentState[1][0] - wheelPositions.getRight());

			}

			leftPower = Math.max(-1, Math.min(1, leftPower));
			rightPower = Math.max(-1, Math.min(1, rightPower));

			Robot.getRobotConfiguration().getDriveTrain().setSpeeds(leftPower, rightPower);
		}
	}

	/**
	 * Calculates parameters used to calculate the speeds of the wheels
	 * 
	 * @param time
	 *            - current time since start
	 * @param step
	 * @return a set of left and right lengths, velocities, and accelerations.
	 */
	private double[][] interpolatePosition(double time) {
		Point[] previousPointSet = new Point[] { currentPath.getLeftPath()[currentStep],
				currentPath.getPathPoints()[currentStep], currentPath.getRightPath()[currentStep] };
		Point[] nextPointSet = new Point[] { currentPath.getLeftPath()[currentStep + 1],
				currentPath.getPathPoints()[currentStep + 1], currentPath.getRightPath()[currentStep + 1] };

		double dTime = nextPointSet[0].getTime() - previousPointSet[0].getTime();
		double p = (nextPointSet[0].getTime() - time) / dTime;
		double q = (time - previousPointSet[0].getTime()) / dTime;
		if (p == 0) {
			return null;
		}

		double lLeft = previousPointSet[0].getLength() * p + nextPointSet[0].getLength() * q;
		double vLeft = previousPointSet[0].getVelocity() * p + nextPointSet[0].getVelocity() * q;
		double aLeft = previousPointSet[0].getAcceleration() * p + nextPointSet[0].getAcceleration() * q;

		double lRight = previousPointSet[2].getLength() * p + nextPointSet[2].getLength() * q;
		double vRight = previousPointSet[2].getVelocity() * p + nextPointSet[2].getVelocity() * q;
		double aRight = previousPointSet[2].getAcceleration() * p + nextPointSet[2].getAcceleration() * q;

		return new double[][] { { lLeft, vLeft, aLeft }, { lRight, vRight, aRight } };
	}

	public void clearMotions() {
		paths.clear();
	}

	public void enableScheduler() {
		if (!running) {
			currentStep = 0;
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
		running = false;
		currentPath = null;
		controller.cancel();
		Robot.getRobotConfiguration().getDriveTrain().setSpeeds(0, 0);
	}
}
