package org.waltonrobotics.motion;

import org.waltonrobotics.metadata.Pose;

/**
 * <p>
 * This Path creates a straight line using the BezierCurve class.
 * </p>
 *
 * @author Russell Newton, Walton Robotics
 * @see BezierCurve
 */
public class Line extends BezierCurve {

  /**
   * Be careful when using this. If your robot's angle is off, the MotionContoller will try to
   * correct for it, so you will not get a straight line.
   *
   * @param vCruise - the cruise velocity of the robot
   * @param aMax - the maximum acceleration of the robot
   * @param startVelocity - the start velocity
   * @param endVelocity - the end velocity
   * @param isBackwards - whether or not to move the robot backwards
   * @param startPose - the starting Pose. Angle doesn't matter
   * @param endPose - the ending Pose. Angle doesn't matter
   */
  public Line(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards,
      Pose startPose, Pose endPose) {
    super(vCruise, aMax, startVelocity, endVelocity, isBackwards, startPose, endPose);
  }

  /**
   * Be careful when using this. If your robot's angle is off, the MotionContoller will try to
   * correct for it, so you will not get a straight line.
   *
   * @param vCruise - the cruise velocity of the robot
   * @param aMax - the maximum acceleration of the robot
   * @param startVelocity - the start velocity
   * @param endVelocity - the end velocity
   * @param isBackwards - whether or not to move the robot backwards
   * @param startPose - the starting Pose. Angle doesn't matter
   * @param distance - how you wish to offset the startingPose from.
   */
  public Line(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards,
      Pose startPose, double distance) {
    this(vCruise, aMax, startVelocity, endVelocity, isBackwards, startPose,
        startPose.offset(distance));
  }

  public String convertToString() {
			/*
		double vCruise,
		double aMax,
		double startVelocity,
		double endVelocity,
		boolean isBackwards,
		List<Pose> controlPoints
		 */

    String className = getClass().getName();

    StringBuilder stringBuilder = new StringBuilder();
    stringBuilder.append(className);
    stringBuilder.append(' ');
    stringBuilder.append(getVCruise());
    stringBuilder.append(' ');
    stringBuilder.append(getAMax());
    stringBuilder.append(' ');
    stringBuilder.append(getStartVelocity());
    stringBuilder.append(' ');
    stringBuilder.append(getEndVelocity());
    stringBuilder.append(' ');
    stringBuilder.append(isBackwards());
    stringBuilder.append(' ');

    double x = getKeyPoints().get(0).getX();
    double y = getKeyPoints().get(0).getY();
    double angle = getKeyPoints().get(0).getAngle();

    stringBuilder.append(String.format("%f,%f,%f", x, y, angle));
    stringBuilder.append(' ');

    x = getKeyPoints().get(1).getX();
    y = getKeyPoints().get(1).getY();
    angle = getKeyPoints().get(1).getAngle();

    stringBuilder.append(String.format("%f,%f,%f", x, y, angle));

    return stringBuilder.toString();
  }
}
