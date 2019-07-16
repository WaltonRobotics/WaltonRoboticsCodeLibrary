package org.waltonrobotics.config;

/**
 * @author Russell Newton
 **/
public interface MotorConfig {

  int getChannel();

  boolean isInverted();

  MotorParameters getMotorParameters();

}
