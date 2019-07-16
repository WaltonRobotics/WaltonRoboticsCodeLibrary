package org.waltonrobotics.config;

import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.RobotPair;

public interface SetSpeeds {

  void setSpeeds(double left, double right);

  RobotPair getWheelPositions();

  /**
   * This method defaults to returning a new Pose(0, 0, 0). This is used in a RamseteController to
   * update the actual position, assuming the useDrivetrainSuppliedPose parameter is set to true. If
   * this is to be used, return a pose created from sensors.
   */
  Pose getSensorCalculatedPose();


}
