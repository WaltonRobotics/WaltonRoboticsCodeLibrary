package org.waltonrobotics.metadata;

/**
 * Holds the cross-track, lag and angle errors
 *
 * @author Russell Newton, Walton Robotics
 */
public class ErrorVector {

  private final double lag;
  private final double crossTrack;
  private final double angle;

  /**
   * @param lag how far behind or in front the actual position is fromm the target position
   * @param crossTrack how far perpendicularly the robot is from the target position
   * @param angle the error between the target angle and the actual angle of the robot
   */
  public ErrorVector(double lag, double crossTrack, double angle) {
    this.lag = lag;
    this.crossTrack = crossTrack;
    this.angle = angle;
  }

  /**
   * @return lag error
   */
  public final double getLag() {
    return lag;
  }

  /**
   * @return xtrack error
   */
  public final double getXTrack() {
    return crossTrack;
  }

  /**
   * @return angle error
   */
  public final double getAngle() {
    return angle;
  }

  @Override
  public String toString() {
    return "ErrorVector{" +
        "lag=" + lag +
        ", crossTrack=" + crossTrack +
        ", angle=" + angle +
        '}';
  }
}
