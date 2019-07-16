package org.waltonrobotics.metadata;

/**
 * Holds information about the left and right sides of the robot at a specific time
 *
 * @author Russell Newton, Walton Robotics
 */
public class RobotPair {

  private final double left;
  private final double right;
  private final double time;

  /**
   * @param left the left power the robot should give the motor at the given time
   * @param right the right power the robot should give the motor at the given time
   * @param time the instance in time
   */
  public RobotPair(double left, double right, double time) {
    this.left = left;
    this.right = right;
    this.time = time;
  }

  /**
   * @return left
   */
  public final double getLeft() {
    return left;
  }

  /**
   * @return right
   */
  public final double getRight() {
    return right;
  }

  /**
   * @return time
   */
  public final double getTime() {
    return time;
  }

  @Override
  public String toString() {
    return "RobotPair{" +
        "left=" + left +
        ", right=" + right +
        ", time=" + time +
        '}';
  }
}
