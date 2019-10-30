package org.waltonrobotics.metadata;

/**
 * Contains information about the robot's org.waltonrobotics.motion at a specific time
 *
 * @author Russell Newton, Walton Robotics
 */
public class MotionData {

  private final Pose actual;
  private final Pose target;
  private final ErrorVector error;
  private final RobotPair powers;
  private final int pathNumber;
  private final MotionState currentMotionState;

  /**
   * @param actual the actual position of the robot
   * @param target the target position of the robot
   * @param error the errors at the specific time
   * @param powers the posers supplied to the motors
   * @param pathNumber the path number the robot is currently going through
   * @param currentMotionState the state the robot is at going through the
   * org.waltonrobotics.motion
   */
  public MotionData(Pose actual, Pose target, ErrorVector error, RobotPair powers,
      int pathNumber,
      MotionState currentMotionState) {
    this.actual = actual;
    this.target = target;
    this.error = error;
    this.powers = powers;
    this.pathNumber = pathNumber;
    this.currentMotionState = currentMotionState;
  }

  /**
   * @return actualPose actual position of the robot
   */
  public final Pose getActualPose() {
    return actual;
  }

  /**
   * @return targetPose the position the robot should be at
   */
  public final Pose getTargetPose() {
    return target;
  }

  /**
   * @return error the errors at the specific time
   * @see ErrorVector
   */
  public final ErrorVector getError() {
    return error;
  }

  /**
   * @return powers and time RobotPair
   * @see RobotPair
   */
  public final RobotPair getPowers() {
    return powers;
  }

  /**
   * @return the state the robot is currently at
   */
  public MotionState getCurrentMotionState() {
    return currentMotionState;
  }

  /**
   * @return the path number
   */
  public int getPathNumber() {
    return pathNumber;
  }

  @Override
  public String toString() {
    return "MotionData{" +
        "actual=" + actual +
        ", target=" + target +
        ", error=" + error +
        ", powers=" + powers +
        ", pathNumber=" + pathNumber +
        ", currentMotionState=" + currentMotionState +
        '}';
  }
}
