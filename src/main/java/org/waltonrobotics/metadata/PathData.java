package org.waltonrobotics.metadata;

/**
 * Holds information about the robot at a specific time in a Path
 *
 * @author Russell Newton, Walton Robotics
 */
public class PathData {

  private final State leftState;
  private final State rightState;
  private final Pose centerPose;
  private final double time;
  private final boolean isBackwards;

  /**
   * @param leftState the left motor state
   * @param rightState the right motor state
   * @param centerPose the center position of the robot in the x,y and angle space
   * @param time the instance in time this was added
   * @param isBackwards if the robot was travelling forwards or backwards
   */
  public PathData(State leftState, State rightState, Pose centerPose, double time,
      boolean isBackwards) {
    this.leftState = leftState;
    this.rightState = rightState;
    this.centerPose = centerPose;
    this.time = time;
    this.isBackwards = isBackwards;
  }

  public PathData(State leftState, State rightState, Pose centerPose, double time) {
    this(leftState, rightState, centerPose, time, false);
  }

  /**
   * Can be used to make a generic PathData without org.waltonrobotics.motion and time
   */
  public PathData(Pose centerPose, boolean isBackwards) {
    this(new State(0, 0, 0), new State(0, 0, 0), centerPose, 0, isBackwards);
  }


  /**
   * Can be used to make a generic PathData without org.waltonrobotics.motion and time
   */
  public PathData(Pose centerPose) {
    this(new State(0, 0, 0), new State(0, 0, 0), centerPose, 0, false);
  }

  /**
   * Can be used to make a generic PathData without org.waltonrobotics.motion
   */
  public PathData(Pose centerPose, double time) {
    this(new State(0, 0, 0), new State(0, 0, 0), centerPose, time, false);
  }

  public boolean isBackwards() {
    return isBackwards;
  }

  /**
   * @return leftState
   */
  public final State getLeftState() {
    return leftState;
  }

  /**
   * @return rightState
   */
  public final State getRightState() {
    return rightState;
  }

  /**
   * @return centerPose
   */
  public final Pose getCenterPose() {
    return centerPose;
  }

  /**
   * @return time
   */
  public final double getTime() {
    return time;
  }

  /**
   * @return the average length of the left and right States
   */
  public final double getLCenter() {
    return (leftState.getLength() + rightState.getLength()) / 2.0;
  }

  @Override
  public String toString() {
    return "PathData{" +
        "leftState=" + leftState +
        ", rightState=" + rightState +
        ", centerPose=" + centerPose +
        ", time=" + time +
        ", isBackwards=" + isBackwards +
        '}';
  }
}
