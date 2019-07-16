package org.waltonrobotics.metadata;

/**
 * The state that the robot is in right now.
 */
public enum MotionState {
  /**
   * The robot is moving through the path org.waltonrobotics.motion
   */
  MOVING,
  /**
   * The robot has finished moving through the path no it is integrating the residual errors in
   * order to finish in the correct spot
   */
  FINISHING,
  /**
   * The robot is waiting for a org.waltonrobotics.motion to be added. It will not move.
   */
  WAITING
}