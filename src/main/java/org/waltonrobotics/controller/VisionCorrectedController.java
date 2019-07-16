package org.waltonrobotics.controller;

import org.waltonrobotics.config.RobotConfig;
import org.waltonrobotics.config.SetSpeeds;
import org.waltonrobotics.metadata.RobotPair;

/**
 * @author Russell Newton
 **/
public class VisionCorrectedController extends MotionController {

  private boolean correctWithVisionHeading = false;

  public VisionCorrectedController(RobotConfig robotConfig, SetSpeeds setSpeeds) {
    super(robotConfig, setSpeeds);
  }

  @Override
  public RobotPair findSpeeds(double time) {
    if(!correctWithVisionHeading) {
      return super.findSpeeds(time);
    } else {
      return new RobotPair(0, 0, time);
    }
  }

  public void setCorrectWithVisionHeading(boolean correctWithVisionHeading) {
    this.correctWithVisionHeading = correctWithVisionHeading;
  }
}
