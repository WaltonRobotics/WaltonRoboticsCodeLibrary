package org.waltonrobotics.config;

/**
 * This class is based off of Team 254's DCMotorTransmission class.
 *
 * @author Russell Newton, Walton Robotics
 **/
public class MotorParameters {

  private static final double kEpsilon = 1e-12;
  private final double speedPerVolt;
  private final double torquePerVolt;
  private final double frictionVoltage;

  /**
   * @param speedPerVolt rad/s/V (no load)
   * @param torquePerVolt N m/V (stall)
   * @param frictionVoltage V
   */
  public MotorParameters(double speedPerVolt, double torquePerVolt, double frictionVoltage) {
    this.speedPerVolt = speedPerVolt;
    this.torquePerVolt = torquePerVolt;
    this.frictionVoltage = frictionVoltage;
  }

  public double getSpeedPerVolt() {
    return speedPerVolt;
  }

  public double getTorquePerVolt() {
    return torquePerVolt;
  }

  public double getFrictionVoltage() {
    return frictionVoltage;
  }

  public double getVoltageFromTorque(double speed, double torque) {
    double frictionVoltage;

    if (speed > kEpsilon) {
      //Forward org.waltonrobotics.motion, rolling friction
      frictionVoltage = this.frictionVoltage;
    } else if (speed < -kEpsilon) {
      //Bacward org.waltonrobotics.motion, rolling friction
      frictionVoltage = -this.frictionVoltage;
    } else if (torque > kEpsilon) {
      //System is static, forward torque
      frictionVoltage = this.frictionVoltage;
    } else if (torque < -kEpsilon) {
      //System is static, backward torque
      frictionVoltage = -this.frictionVoltage;
    } else {
      //System is idle
      return 0.0;
    }

    return torque / torquePerVolt + speed / speedPerVolt + frictionVoltage;
  }

  public double getVoltageFromSpeed(double speed) {
    return speed / speedPerVolt;
  }
}
