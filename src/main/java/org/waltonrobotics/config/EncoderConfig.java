package org.waltonrobotics.config;

public interface EncoderConfig {

  double getDistancePerPulse();

  int getChannel1();

  int getChannel2();

  boolean isInverted();
}
