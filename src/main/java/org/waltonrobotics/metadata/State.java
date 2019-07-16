package org.waltonrobotics.metadata;

/**
 * Holds an encoder length, velocity, and acceleration
 *
 * @author Russell Newton, Walton Robotics
 */
public class State {

  private final double length;
  private final double velocity;
  private final double acceleration;
  private final double jerk;

  /**
   * @param length how far the robot has to go
   * @param velocity the velocity the robot should be
   * @param acceleration the acceleration the robot should be
   */
  public State(double length, double velocity, double acceleration) {
    this(length, velocity, acceleration, 0);
  }

  /**
   * @param length how far the robot has to go
   * @param velocity the velocity the robot should be
   * @param acceleration the acceleration the robot should be
   * @param jerk the jerk the robot should be at
   */
  public State(double length, double velocity, double acceleration, double jerk) {
    this.length = length;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.jerk = jerk;
  }

  public static final State calculateConstants(State previous, double wheelDistance,
      double deltaTime) {
    double velocity = (wheelDistance - previous.getLength()) / deltaTime;
    double acceleration = (velocity - previous.getVelocity()) / deltaTime;
    double jerk = (acceleration - previous.getAcceleration()) / deltaTime;

    return new State(wheelDistance, velocity, acceleration, jerk);
  }

  public double getJerk() {
    return jerk;
  }

  /**
   * @return length
   */
  public final double getLength() {
    return length;
  }

  /**
   * @return velocity
   */
  public final double getVelocity() {
    return velocity;
  }

  /**
   * @return acceleration
   */
  public final double getAcceleration() {
    return acceleration;
  }

  @Override

  public String toString() {
    return "State{" +
        "length=" + length +
        ", velocity=" + velocity +
        ", acceleration=" + acceleration +
        '}';
  }
}
