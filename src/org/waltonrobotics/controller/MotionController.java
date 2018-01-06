package org.waltonrobotics.controller;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.Robot;

/**
 * Sends power to the wheels
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
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
	private State[] staticState;
	private double startTime;

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

			State[] currentState;
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
				Robot.getRobotConfiguration().getDriveTrain().reset();
				currentStep = 0;
				//FIXME startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();\
				staticState = new State[] {
						new State(Robot.getRobotConfiguration().getDriveTrain().getWheelPositions().getLeft(), 0, 0),
						new State(Robot.getRobotConfiguration().getDriveTrain().getWheelPositions().getRight(), 0, 0) };
				return;
			}

			synchronized (this) {
				// feed forward
				leftPower += (kVoltage * currentState[0].getVelocity() + kScaling)
						+ kCurrent * currentState[0].getAcceleration();
				rightPower += (kVoltage * currentState[1].getVelocity() + kScaling)
						+ kCurrent * currentState[1].getAcceleration();
				// feed back
				leftPower += kPower * (currentState[0].getLength() - wheelPositions.getLeft());
				rightPower += kPower * (currentState[1].getLength() - wheelPositions.getRight());

			}

			leftPower = Math.max(-1, Math.min(1, leftPower));
			rightPower = Math.max(-1, Math.min(1, rightPower));

			Robot.getRobotConfiguration().getDriveTrain().setSpeeds(leftPower, rightPower);
		}
	}

	/**
	 * Calculates parameters for wheel powers based off of location between steps
	 * 
	 * @param time
	 *            - current time since start
	 * @param step
	 * @return a set of left and right lengths, velocities, and accelerations.
	 */
	private State[] interpolatePosition(double time) {
		Point previousLeft = currentPath.getLeftPath()[currentStep];
		Point previousRight = currentPath.getRightPath()[currentStep];
		Point nextLeft = currentPath.getLeftPath()[currentStep + 1];
		Point nextRight = currentPath.getRightPath()[currentStep + 1];

		// Difference in time
		double dTime = nextLeft.getTime() - previousLeft.getTime();
		// Ratio from position to next point
		double ratioPosToNext = (nextLeft.getTime() - time) / dTime;
		// Ratio from previous point to position
		double rationPrevToPos = (time - previousLeft.getTime()) / dTime;
		if (ratioPosToNext == 0) {
			return null;
		}

		// New values are the average of the previous and next point values
		// (Average = sum(value * ratio))
		double lLeft = previousLeft.getLength() * ratioPosToNext + nextLeft.getLength() * rationPrevToPos;
		double vLeft = previousLeft.getVelocity() * ratioPosToNext + nextLeft.getVelocity() * rationPrevToPos;
		double aLeft = previousLeft.getAcceleration() * ratioPosToNext + nextLeft.getAcceleration() * rationPrevToPos;

		double lRight = previousRight.getLength() * ratioPosToNext + nextRight.getLength() * rationPrevToPos;
		double vRight = previousRight.getVelocity() * ratioPosToNext + nextRight.getVelocity() * rationPrevToPos;
		double aRight = previousRight.getAcceleration() * ratioPosToNext
				+ nextRight.getAcceleration() * rationPrevToPos;

		return new State[] { new State(lLeft, vLeft, aLeft), new State(lRight, vRight, aRight) };
	}

	/**
	 * Removes all queued motions
	 */
	public void clearMotions() {
		paths.clear();
	}

	/**
	 * Starts the queue of motions
	 */
	public void enableScheduler() {
		if (!running) {
			currentStep = 0;
			//FIXME startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
			staticState = new State[] {
					new State(Robot.getRobotConfiguration().getDriveTrain().getWheelPositions().getLeft(), 0, 0),
					new State(Robot.getRobotConfiguration().getDriveTrain().getWheelPositions().getRight(), 0, 0) };
			Path newPath = paths.poll();
			if (newPath != null) {
				controller.scheduleAtFixedRate(new MotionTask(), 0L, (long) period);
				running = true;
			}
		}
	}

	/**
	 * 
	 * @return Whether or not the queue has ended
	 */
	public boolean isFinished() {
		return false; // FIXME do this
	}

	/**
	 * 
	 * @return Whether or not a motion is running
	 */
	public boolean isRunning() {
		return running;
	}

	/**
	 * Pauses the motions,
	 */
	public void stopScheduler() {
		running = false;
		currentPath = null;
		controller.cancel();
		Robot.getRobotConfiguration().getDriveTrain().setSpeeds(0, 0);
	}
}
