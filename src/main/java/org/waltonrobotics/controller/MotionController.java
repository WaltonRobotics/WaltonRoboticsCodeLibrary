package org.waltonrobotics.controller;

import java.util.Collections;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.LinkedBlockingDeque;
import java.util.function.Supplier;
import javax.annotation.Nullable;
import org.waltonrobotics.config.RobotConfig;
import org.waltonrobotics.config.SetSpeeds;
import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.ErrorVector;
import org.waltonrobotics.metadata.MotionData;
import org.waltonrobotics.metadata.MotionState;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.RobotPair;
import org.waltonrobotics.metadata.State;
import org.waltonrobotics.motion.Path;

/**
 * Controls Path motions
 *
 * @author Russell Newton, Walton Robotics
 */
public class MotionController {

  private final Queue<Path> paths = new LinkedBlockingDeque<>();
  private final int period;
  private final MotionLogger motionLogger;
  private final Timer controller;
  private final List<PathData> history = new LinkedList<>();
  protected final SetSpeeds setSpeeds;
  private final Supplier<Boolean> usingCamera;
  protected RobotConfig robotConfig;
  protected PathData targetPathData;
  protected ErrorVector errorVector;
  private boolean running;
  private Path currentPath;
  private PathData staticPathData;
  private Pose actualPosition;
  private RobotPair previousLengths;
  private double pathStartTime;
  private ListIterator<PathData> pdIterator;
  private PathData pdPrevious;
  private PathData pdNext;
  private RobotPair powers;
  private TimerTask currentTimerTask;
  private MotionState currentMotionState = MotionState.WAITING;
  private double integratedLagError;
  private double integratedAngleError;
  private int pathNumber;
  private CameraReader cameraReader = new CameraReader();
  private final boolean reverseAngle;

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   * @param robotWidth - the robot width from the outside of the wheels
   * @param motionLogger - the MotionLogger from the org.waltonrobotics.AbstractDrivetrain
   */
  public MotionController(RobotConfig robotConfig, double robotWidth, MotionLogger motionLogger,
      SetSpeeds setSpeeds,
      Supplier<Boolean> usingCamera) {
    this.setSpeeds = setSpeeds;
    this.usingCamera = usingCamera;
    this.reverseAngle = robotConfig.reverseAngleCalculation();
    running = false;
    Path.setRobotWidth(robotWidth);

    this.motionLogger = motionLogger;

    controller = new Timer();
    period = 5;

    RobotPair wheelPositions = setSpeeds.getWheelPositions();
    staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
        new State(wheelPositions.getRight(), 0, 0), new Pose(0, 0, 0), 0, true);

    this.robotConfig = robotConfig;

    pathNumber = 0;
  }

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   * @param motionLogger - the MotionLogger from the org.waltonrobotics.AbstractDrivetrain
   */
  public MotionController(RobotConfig robotConfig, MotionLogger motionLogger, SetSpeeds setSpeeds,
      Supplier<Boolean> usingCamera) {
    this(robotConfig, robotConfig.getRobotWidth(), motionLogger, setSpeeds, usingCamera);
  }

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   * @param motionLogger - the MotionLogger from the org.waltonrobotics.AbstractDrivetrain
   */
  public MotionController(RobotConfig robotConfig, MotionLogger motionLogger, SetSpeeds setSpeeds) {
    this(robotConfig, robotConfig.getRobotWidth(), motionLogger, setSpeeds, () -> false);
  }

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   */
  public MotionController(RobotConfig robotConfig, SetSpeeds setSpeeds,
      Supplier<Boolean> usingCamera) {
    this(robotConfig, robotConfig.getRobotWidth(), new MotionLogger(), setSpeeds, usingCamera);
  }


  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   */
  public MotionController(RobotConfig robotConfig, SetSpeeds setSpeeds) {
    this(robotConfig, robotConfig.getRobotWidth(), new MotionLogger(), setSpeeds, () -> false);
  }

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   */
  public MotionController(RobotConfig robotConfig) {
    this(robotConfig, () -> false);
  }

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods from
   */
  public MotionController(RobotConfig robotConfig, Supplier<Boolean> usingCamera) {
    this(robotConfig, robotConfig.getRobotWidth(), new MotionLogger(), new SetSpeeds() {
      @Override
      public void setSpeeds(double left, double right) {

      }

      @Override
      public RobotPair getWheelPositions() {
        return null;
      }

      @Override
      public double getSensorCalculatedHeading() {
        return 0;
      }
    }, usingCamera);
  }

  /**
   * Updates where the robot thinks it is, based off of the encoders and any sensor input
   */
  public Pose updateActualPosition(RobotPair wheelPositions,
      RobotPair previousWheelPositions,
      Pose estimatedActualPosition, @Nullable Double sensorHeading, @Nullable Pose sensorPose) {
    double arcLeft = wheelPositions.getLeft() - previousWheelPositions.getLeft();
    double arcRight = wheelPositions.getRight() - previousWheelPositions.getRight();
    double dAngle = (arcRight - arcLeft) / Path.getRobotWidth() * (reverseAngle ? -1 : 1);
    double arcCenter = (arcRight + arcLeft) / 2.0;
    double dX;
    double dY;

    double currentAngle;
    if(sensorHeading == null) {
      currentAngle = estimatedActualPosition.getAngle();
    } else {
      currentAngle = sensorHeading;
    }

    if (Math.abs(dAngle) < 0.01) {
      dX = arcCenter * StrictMath.cos(currentAngle);
      dY = arcCenter * StrictMath.sin(currentAngle);
    } else {
      double xPrime = (arcCenter / dAngle) * StrictMath.sin(dAngle);
      double yPrime = (arcCenter / dAngle) * (1.0 - StrictMath.cos(dAngle));

      dX = (xPrime * StrictMath.cos(currentAngle))
          - (yPrime * StrictMath.sin(currentAngle));

      dY = ((xPrime * StrictMath.sin(currentAngle)))
          + ((yPrime * StrictMath.cos(currentAngle)));
    }

    estimatedActualPosition = estimatedActualPosition.offset(dX, dY, dAngle);
    if(sensorHeading != null) {
      estimatedActualPosition = new Pose(estimatedActualPosition.getX(),
          estimatedActualPosition.getY(), sensorHeading);
    }
    if(sensorPose != null) {
      estimatedActualPosition = new Pose(sensorPose.getX(), sensorPose.getY(),
          estimatedActualPosition.getAngle());
    }

    return estimatedActualPosition;
  }

  public Pose updateActualPosition(RobotPair wheelPositions,
      RobotPair previousWheelPositions,
      Pose estimatedActualPosition) {
    return updateActualPosition(wheelPositions, previousWheelPositions, estimatedActualPosition,
        null, null);
  }

  /**
   * Finds the current lag and cross track ErrorVector
   */
  public static ErrorVector findCurrentError(PathData targetPathData, Pose actualPose) {
    Pose targetPose = targetPathData.getCenterPose();
    double dX = targetPose.getX() - actualPose.getX();
    double dY = targetPose.getY() - actualPose.getY();
    double angle = targetPose.getAngle();
    // error in direction facing
    double lagError = (dX * StrictMath.cos(angle)) + (dY * StrictMath.sin(angle));
    // error perpendicular to direction facing

    double crossTrackError = (-dX * StrictMath.sin(angle)) + (dY * StrictMath.cos(angle));
    // the error of the current angle
    double angleError = targetPose.getAngle() - actualPose.getAngle();

    if (targetPathData.isBackwards()) {
      crossTrackError *= -1.0;
    }

    if (angleError > Math.PI) {
      angleError -= 2.0 * Math.PI;
    } else if (angleError < -Math.PI) {
      angleError += 2.0 * Math.PI;
    }
    return new ErrorVector(lagError, crossTrackError, angleError);
  }

  public CameraReader getCameraReader() {
    return cameraReader;
  }

  public void setRobotConfig(RobotConfig robotConfig) {
    this.robotConfig = robotConfig;
  }

  public MotionLogger getMotionLogger() {
    return motionLogger;
  }

  public int getPathNumber() {
    return pathNumber;
  }

  /**
   * Adds a path to the path queue
   *
   * @param paths - the paths to add to the queue
   */
  public final void addPaths(Path... paths) {
    Collections.addAll(this.paths, paths);
  }

  public void findCurrentPath(RobotPair wheelPositions) {
    if (currentPath != null) {
      targetPathData = interpolate(wheelPositions);

      if (currentPath.isFinished()) {
        System.out.println("Current path is finished");
        Deque<PathData> temp = currentPath.getPathData();
        currentPath = paths.poll();

        integratedLagError = 0;
        integratedAngleError = 0;

        if (currentPath != null) {
          System.out.println("Getting new path");
          double time = temp.getLast().getTime() - temp.getFirst().getTime();

          //Used to allow smooth transition between motions not making assumption that it finishes perfectly on time
          pathStartTime = time + pathStartTime;

          pdIterator = currentPath.getPathData().listIterator();
          pdPrevious = targetPathData = pdIterator.next();
          pdNext = pdIterator.next();

          targetPathData = interpolate(wheelPositions);

          currentMotionState = MotionState.MOVING;
          pathNumber += 1;
        } else {
          System.out.println("Done with motions! :)");

          staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
              new State(wheelPositions.getRight(), 0, 0),
              targetPathData.getCenterPose(), wheelPositions.getTime(),
              targetPathData.isBackwards());
          targetPathData = staticPathData;
          currentMotionState = MotionState.FINISHING;
        }
      }
    } else {
      // if there is absolutely no more paths at the moment says to not move

      currentPath = paths.poll();
      if (currentPath != null) {
        System.out.println("Getting initial path");
//					actualPosition = currentPath.getPathData().get(0).getCenterPose();
        pathStartTime = wheelPositions.getTime();
        pdIterator = currentPath.getPathData().listIterator();
        pdPrevious = targetPathData = pdIterator.next();
        pdNext = pdIterator.next();

        currentMotionState = MotionState.MOVING;
        targetPathData = interpolate(wheelPositions);

        integratedLagError = 0;
        integratedAngleError = 0;

        pathNumber += 1;
      } else {
//					System.out.println("No initial path not moving");
        targetPathData = staticPathData;
        System.out.println("No path available");
      }
    }

  }

  /**
   * Calculates the powers to send to the wheels
   *
   * @return a RobotPair with the powers and the time
   */
  private synchronized RobotPair calculateSpeeds(RobotPair wheelPositions) {
    if (running) {
      findCurrentPath(wheelPositions);
      actualPosition = updateActualPosition(wheelPositions, previousLengths, actualPosition);

      System.out.println("Current Position: " + actualPosition);
      System.out.println("Target Position: " + targetPathData.getCenterPose());

      history.add(new PathData(actualPosition, wheelPositions.getTime()));

      previousLengths = wheelPositions;

      errorVector = findCurrentError(targetPathData, actualPosition);

      if (isUsingCamera()) {
        errorVector = handleCameraData();
      }

      return findSpeeds(wheelPositions.getTime());
    }

    return new RobotPair(0, 0, wheelPositions.getTime());
  }

  private boolean isUsingCamera() {
    return usingCamera.get();
  }


  public RobotPair findSpeeds(double time) {
    double leftPower = 0;
    double rightPower = 0;

    System.out.printf("kV %f, kK %f, kAcc %f, kS %f, kAng %f, kL %f%n", robotConfig.getKV(),
        robotConfig.getKK(),
        robotConfig.getKAcc(), robotConfig.getKS(), robotConfig.getKAng(), robotConfig.getKL());
    double centerPower = 0;
    double steerPower = 0;
    if (currentMotionState == MotionState.MOVING) {
      // feed forward

      leftPower += ((robotConfig.getKV() * targetPathData.getLeftState().getVelocity())
          + (robotConfig.getKK() * Math.signum(targetPathData.getLeftState().getVelocity())))
          + (robotConfig.getKAcc() * targetPathData.getLeftState().getAcceleration());
      rightPower += ((robotConfig.getKV() * targetPathData.getRightState().getVelocity())
          + (robotConfig.getKK() * Math.signum(targetPathData.getRightState().getVelocity())))
          + (robotConfig.getKAcc() * targetPathData.getRightState().getAcceleration());
      // feed back
      double steerPowerXTE = robotConfig.getKS() * errorVector.getXTrack();
      double steerPowerAngle =
          robotConfig.getKAng() * errorVector
              .getAngle(); //error angle must be negative because it is target-actual
      double centerPowerLag = robotConfig.getKL() * errorVector.getLag();

      System.out.printf("steerPowerXTE: %f steerPowerAngle: %f centerPowerLag: %f%n", steerPowerXTE,
          steerPowerAngle,
          centerPowerLag);

      centerPower = ((leftPower + rightPower) / 2.0) + centerPowerLag;
      steerPower = Math.max(-1.0,
          Math.min(1.0, ((rightPower - leftPower) / 2.0) + steerPowerXTE + steerPowerAngle));
      centerPower = Math
          .max(-1.0 + Math.abs(steerPower),
              Math.min(1.0 - Math.abs(steerPower), centerPower));
    }
    if ((currentMotionState == MotionState.FINISHING) || isClose(1.0)) {
//          to give the extra oomph when finished the path but there is a little bit more to do//FIXME left, right powers somehow manage to be greater than 1

      if ((time - staticPathData.getTime()) >= 2.0) {
        currentMotionState = MotionState.WAITING;
      }

      integratedLagError += robotConfig.getILag() * errorVector.getLag();
      integratedAngleError += robotConfig.getIAng() * errorVector.getAngle();

      integratedAngleError = Math.max(Math.min(0.5, integratedAngleError), -0.5);
      integratedLagError = Math.max(Math.min(0.5, integratedLagError), -0.5);

      steerPower += integratedAngleError;
      centerPower += integratedLagError;
    }

    if (currentMotionState == MotionState.WAITING) {
      steerPower = 0;
      centerPower = 0;
    }

    System.out.println(currentMotionState);
    System.out.printf("Left Power %f \t Right Power %f%n", centerPower - steerPower,
        centerPower + steerPower);
    return new RobotPair(centerPower - steerPower, centerPower + steerPower,
        time);
  }

  private ErrorVector handleCameraData() {
    CameraData robotToCameraPosition = getCurrentCameraData();
    PathData pathData = findClosestPointInHistory(robotToCameraPosition.getTime());
//        TODO interpret
    trimHistory(pathData);
    ErrorVector errorVector = findCurrentError(pathData, robotToCameraPosition.getCameraPose());

    return new ErrorVector(errorVector.getLag() + this.errorVector.getLag(),
        errorVector.getXTrack() + this.errorVector.getXTrack(),
        errorVector.getAngle() + this.errorVector.getAngle()
    );
  }

  private void trimHistory(PathData pathData) {
    for (PathData pathData1 : history) {
      if (pathData1.getTime() < pathData.getTime()) {
        history.remove(pathData1);
      }
    }
  }

  private PathData findClosestPointInHistory(double pointTime) {
    PathData previousPathData = history.get(0);

    for (int i = history.size() - 1; i >= 1; i--) {
      previousPathData = history.get(i - 1);

      if ((previousPathData.getTime() >= pointTime) && (history.get(i).getTime() < pointTime)) {
        break;
      }
    }

    return previousPathData;
  }

  /**
   * Finds the target x, y, angle, velocityLeft, and velocityRight
   *
   * @return a new MotionData with the interpolated data
   */
  private PathData interpolate(RobotPair wheelPositions) {
    double currentTime = wheelPositions.getTime() - pathStartTime;
    while (currentTime > pdNext.getTime()) {
      if (pdIterator.hasNext()) {
        pdPrevious = pdNext;
        pdNext = pdIterator.next();
      } else {
        pdPrevious = pdNext;
        currentPath.setFinished(true);
        return pdNext;
      }
    }

    double timePrevious = pdPrevious.getTime();
    double timeNext = pdNext.getTime();
    double dTime = timeNext - timePrevious;
    double rctn =
        (timeNext - currentTime) / dTime; // Ratio of the current time to the next pose time
    double rltc =
        (currentTime - timePrevious)
            / dTime; // Ratio of the previous time to the current pose
    // time

    double lengthLeft = ((pdPrevious.getLeftState().getLength()) * rctn)
        + ((pdNext.getLeftState().getLength()) * rltc);
    double lengthRight = ((pdPrevious.getRightState().getLength()) * rctn)
        + ((pdNext.getRightState().getLength()) * rltc);

    // Current pose is made from the weighted average of the x, y, and angle values
    double x =
        (pdPrevious.getCenterPose().getX() * rctn) + (pdNext.getCenterPose().getX() * rltc);
    double y =
        (pdPrevious.getCenterPose().getY() * rctn) + (pdNext.getCenterPose().getY() * rltc);
    double angle =
        (pdPrevious.getCenterPose().getAngle() * rctn) + (pdNext.getCenterPose().getAngle()
            * rltc);

    State left = new State(lengthLeft, pdNext.getLeftState().getVelocity(),
        pdNext.getLeftState().getAcceleration());
    State right = new State(lengthRight, pdNext.getRightState().getVelocity(),
        pdNext.getRightState().getAcceleration());
    Pose centerPose = new Pose(x, y, angle);
    return new PathData(left, right, centerPose, currentTime, currentPath.isBackwards());
  }

  /**
   * Removes all queued motions
   */
  public final synchronized void clearMotions() {
    paths.clear();
  }

  /**
   * Starts the queue of motions
   */
  public final synchronized void enableScheduler() {
    if (!running) {
      System.out.println("Enabling scheduler");
      System.out.println(actualPosition);
//			actualPosition = starting;
      if (actualPosition == null) {
        System.out
            .println("Starting position might not have been set setting actual position to 0,0,0");
        actualPosition = new Pose(0, 0, 0);
      }

      previousLengths = setSpeeds.getWheelPositions();

      staticPathData = new PathData(
          new State(setSpeeds.getWheelPositions().getLeft(), 0, 0),
          new State(setSpeeds.getWheelPositions().getRight(), 0, 0),
          actualPosition,
          0, true);

      targetPathData = staticPathData;

      currentTimerTask = new MotionTask();
      controller.schedule(currentTimerTask, 0L, period);
      currentMotionState = MotionState.WAITING;
      running = true;
    }
  }

  public CameraData getCurrentCameraData() {
    cameraReader.run();
    return cameraReader.getCameraData();
  }

  /**
   * @return Whether or not the queue has ended
   */
  public final boolean isFinished() {
    return (currentPath == null) && (currentMotionState == MotionState.FINISHING);
  }

  /**
   * @return Whether or not a org.waltonrobotics.motion is running
   */
  public final boolean isRunning() {
    return running;
  }

  /**
   * @return Percent of the current Path that the robot is at, based off of the time
   */
  public double getPercentDone(Path pathToUse) {
    if (currentPath.equals(pathToUse)) {
      double currentTime = setSpeeds.getWheelPositions().getTime() - pathStartTime;
      double endTime = currentPath.getPathData().getLast().getTime();
      return currentTime / endTime;
    }
    return -1.0;
  }

  /**
   * Pauses the motions,
   */
  public final synchronized void stopScheduler() {
    if (running) {
      System.out.println("Disabling scheduler");
      running = false;
      currentTimerTask.cancel();
      controller.purge();
      currentPath = null;
      setSpeeds.setSpeeds(0, 0);
      pathNumber = 0;
    }
  }

  public boolean isClose(double closeTime) {
    return (currentPath != null) && (targetPathData != null)
        && (((currentPath.getPathData().getLast().getTime() + pathStartTime) - targetPathData
        .getTime())
        <= closeTime);
  }

  @Override
  public String toString() {
    return "MotionController{" +
        "robotConfig=" + robotConfig +
        ", paths=" + paths +
        ", period=" + period +
        ", motionLogger=" + motionLogger +
        ", org.waltonrobotics.controller=" + controller +
        ", running=" + running +
        ", currentPath=" + currentPath +
        ", staticPathData=" + staticPathData +
        ", actualPosition=" + actualPosition +
        ", targetPathData=" + targetPathData +
        ", previousLengths=" + previousLengths +
        ", pathStartTime=" + pathStartTime +
        ", pdIterator=" + pdIterator +
        ", pdPrevious=" + pdPrevious +
        ", pdNext=" + pdNext +
        ", errorVector=" + errorVector +
        ", powers=" + powers +
        ", currentTimerTask=" + currentTimerTask +
        ", currentMotionState=" + currentMotionState +
        ", integratedLagError=" + integratedLagError +
        ", integratedAngleError=" + integratedAngleError +
        ", pathNumber=" + pathNumber +
        '}';
  }

  public void setStartPosition(Pose startingPosition) {

    actualPosition = startingPosition;
  }

  public List<ErrorVector> getErrorVectorList() {
    return motionLogger.getErrorVectorList();
  }

  /**
   * 1 Runs the calculations with a TimerTask
   *
   * @author Russell Newton, WaltonRobotics
   */
  private class MotionTask extends TimerTask {

    @Override
    public final void run() {
      if ((robotConfig.getKK() == 0) && (robotConfig.getKV() == 0) && (robotConfig.getKL() == 0)) {
        System.out
            .println("Please make KK, KV or KL, not equal 0 otherwise the robot will not move");
      }

      RobotPair wheelPositions = setSpeeds.getWheelPositions();

      powers = calculateSpeeds(wheelPositions);

      setSpeeds.setSpeeds(powers.getLeft(), powers.getRight());

      motionLogger.addMotionData(
          new MotionData(actualPosition, targetPathData.getCenterPose(), errorVector,
              powers, pathNumber, currentMotionState));
    }
  }
}
