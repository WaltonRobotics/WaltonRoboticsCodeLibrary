package org.waltonrobotics;

import edu.wpi.first.wpilibj.command.Subsystem;
import java.util.function.Supplier;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.config.RobotConfig;
import org.waltonrobotics.config.SetSpeeds;
import org.waltonrobotics.controller.MotionController;
import org.waltonrobotics.controller.MotionLogger;
import org.waltonrobotics.controller.RamseteController;
import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.RobotPair;
import org.waltonrobotics.metadata.State;
import org.waltonrobotics.motion.Path;

/**
 * Extend this in your drivetrain, and use the methods inside to set up spline motions
 *
 * @author Russell Newton, Marius Juston, Walton Robotics
 */
public abstract class AbstractDrivetrain extends Subsystem {
//TODO use the java.org.waltonrobotics.util.Properties class to save and load the drivetrain constants to a file

  private final MotionController controller;
  private final Supplier<Boolean> usingCamera;
  private final ControllerType controllerType;
  private final long period = 5L;
  private RobotConfig robotConfig;
  private Pose actualPosition = new Pose(0, 0, 0);
  private RobotPair previousLengths;
  private double actualPositionTime;
  private PathData currentState;
  private PathData previousState;

  public AbstractDrivetrain(RobotConfig robotConfig) {
    this(robotConfig, () -> false, ControllerType.POWERUP, false, false);
  }

  public AbstractDrivetrain(RobotConfig robotConfic, ControllerType controllerType) {
    this(robotConfic, () -> false, controllerType, false, false);
  }

  public AbstractDrivetrain(RobotConfig robotConfic, Supplier<Boolean> usingCamera) {
    this(robotConfic, usingCamera, ControllerType.POWERUP, false, false);
  }

  /**
   * Create the static drivetrain after creating the org.waltonrobotics.motion logger so you can use
   * the MotionController
   */
  public AbstractDrivetrain(RobotConfig robotConfig, Supplier<Boolean> usingCamera,
      ControllerType controllerType, boolean useMotorProfiles,
      boolean useDrivetrainSuppliedHeading) {
    this.robotConfig = robotConfig;
    this.controllerType = controllerType;
    if ((robotConfig.getKK() == 0) && (robotConfig.getKV() == 0) && (robotConfig.getKL() == 0)) {
      System.out.println("Please make KK, KV or KL, not equal 0 otherwise the robot will not move");
    }

    AbstractDrivetrain drivetrain = this;
    this.usingCamera = usingCamera;

    switch (controllerType) {
      case RAMSETE:
        controller =
            new RamseteController(robotConfig, new SetSpeeds() {
              @Override
              public void setSpeeds(double left, double right) {
                drivetrain.setSpeeds(left, right);
              }

              @Override
              public RobotPair getWheelPositions() {
                return drivetrain.getWheelPositions();
              }

              @Override
              public double getSensorCalculatedHeading() {
                return getNavXHeading();
              }
            }, useMotorProfiles, useDrivetrainSuppliedHeading, usingCamera);
        break;
      default:
        controller =
            new MotionController(robotConfig, new SetSpeeds() {
              @Override
              public void setSpeeds(double left, double right) {
                drivetrain.setSpeeds(left, right);
              }

              @Override
              public RobotPair getWheelPositions() {
                return drivetrain.getWheelPositions();
              }

              @Override
              public double getSensorCalculatedHeading() {
                return getNavXHeading();
              }
            }, usingCamera);
        break;
    }

    SimpleMotion.setDrivetrain(this);
    previousLengths = getWheelPositions();

    previousState = new PathData(
        new State(previousLengths.getLeft(), 0, 0),
        new State(previousLengths.getRight(), 0, 0),
        actualPosition, previousLengths.getTime());

    currentState = previousState;

    setEncoderDistancePerPulse();
  }


  @Override
  public void periodic() {
    //		Gets the current predicted actual position
    RobotPair wheelPosition = getWheelPositions();
//    actualPosition = MotionController
//        .updateActualPosition(wheelPosition, previousLengths, actualPosition);
    actualPositionTime = wheelPosition.getTime();

//		Found change in time between the different periodic calls
    double deltaTime = wheelPosition.getTime() - previousState.getTime();

////		Velocity is the change of distance over time
//		double lVelocity = (wheelPosition.getLeft() - previousState.getLeftState().getLength()) / deltaTime;
//		double rVelocity = (wheelPosition.getRight() - previousState.getRightState().getLength()) / deltaTime;
//
////		Acceleration is the change of velocity over time
//		double lAcceleration = (lVelocity - previousState.getLeftState().getVelocity()) / deltaTime;
//		double rAcceleration = (rVelocity - previousState.getRightState().getVelocity()) / deltaTime;
//
////		Jerk is the change of acceleration over time
//		double lJerk = (lAcceleration - previousState.getLeftState().getAcceleration()) / deltaTime;
//		double rJerk = (rAcceleration - previousState.getRightState().getAcceleration()) / deltaTime;

    currentState = new PathData(
        State.calculateConstants(previousState.getLeftState(), wheelPosition.getLeft(), deltaTime),
        State
            .calculateConstants(previousState.getRightState(), wheelPosition.getRight(), deltaTime),
//			new State(wheelPosition.getLeft(), lVelocity, lAcceleration, lJerk),
//			new State(wheelPosition.getRight(), rVelocity, rAcceleration, rJerk),
        actualPosition,
        actualPositionTime);

    previousState = currentState;
    previousLengths = wheelPosition;
  }

  public MotionController getController() {
    return controller;
  }

  public RobotConfig getRobotConfig() {
    return robotConfig;
  }

  public void setRobotConfig(RobotConfig robotConfig) {
    this.robotConfig = robotConfig;
    controller.setRobotConfig(robotConfig);
  }

  public boolean isUsingCamera() {
    return usingCamera.get();
  }

  public CameraData getCurrentCameraData() {
    return controller.getCurrentCameraData();
  }

  public abstract RobotPair getWheelPositions();

  /**
   * Reset the encoders here
   */
  public abstract void reset();

  /**
   * @return whether or not the MotionController is running
   */
  public final boolean getControllerStatus() {
    return controller.isRunning();
  }

  /**
   * Starts the MotionController
   */
  public final void startControllerMotion(Pose startPosition) {
    controller.setStartPosition(startPosition);
    controller.enableScheduler();
  }

  /**
   * Starts the MotionController
   */
  public final void startControllerMotion() {
    controller.setStartPosition(actualPosition);
    controller.enableScheduler();
  }

  /**
   * Cancels the MotionController
   */
  public final void cancelControllerMotion() {
    controller.stopScheduler();
  }

  /**
   * Clears the current queue
   */
  public final void clearControllerMotions() {
    controller.clearMotions();
  }

  /**
   * @param paths - paths to add to the MotionController queue
   */
  public final void addControllerMotions(Path... paths) {
    controller.addPaths(paths);
  }

  /**
   * @return if the robot has completed all motions
   */
  public final boolean isControllerFinished() {
    return controller.isFinished();
  }

  /**
   * Set the motor speeds here
   */
  public abstract void setSpeeds(double leftPower, double rightPower);

  /**
   * Set the encoder distances per pulse here
   */
  public abstract void setEncoderDistancePerPulse();

  public MotionLogger getMotionLogger() {
    return controller.getMotionLogger();
  }

  public double getPercentPathDone(Path path) {
    return controller.getPercentDone(path);
  }

  /**
   * Returns the approximate actual location of the robot from the origin (0,0)
   *
   * @return the approximate location of the robot from (0,0)
   */
  public Pose getActualPosition() {
    return actualPosition;
  }

  /**
   * Sets the start location of the robot instead of the origin of (0,0)
   */
  public void setStartingPosition(Pose startingPosition) {
    controller.setStartPosition(startingPosition);
    actualPosition = startingPosition;
    previousLengths = getWheelPositions();
    previousState = new PathData(startingPosition);
  }

  @Override
  public String toString() {
    return "org.waltonrobotics.AbstractDrivetrain{" +
        "org.waltonrobotics.controller=" + controller +
        ", actualPosition=" + actualPosition +
        ", previousLengths=" + previousLengths +
        ", actualPositionTime=" + actualPositionTime +
        ", currentState=" + currentState +
        ", previousState=" + previousState +
        '}';
  }

  /**
   * Gets the actual position time on the robot
   */
  public double getActualPositionTime() {
    return actualPositionTime;
  }

  /**
   * Gets the current state of the robot (wheel encoder distances, velocities, accelerations and
   * jerks), its position (x, y, angle) and the time it was calculated
   */
  public PathData getCurrentRobotState() {
    return currentState;
  }

  public int getPathNumber() {
    return controller.getPathNumber();
  }

  /**
   * Override this if you're going to be using a RamseteController with navX feedback. It defaults
   * to returning a Pose(0, 0, 0)
   *
   * @return a pose created form the navX data
   */
  protected double getNavXHeading() {
    return 0;
  }

  public enum ControllerType {
    POWERUP,
    RAMSETE
  }
}
