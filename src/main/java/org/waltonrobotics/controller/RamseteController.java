package org.waltonrobotics.controller;

import java.util.function.Supplier;
import org.waltonrobotics.config.RobotConfig;
import org.waltonrobotics.config.SetSpeeds;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.RobotPair;
import org.waltonrobotics.motion.Path;

/**
 * A RamseteController is very similar to our other MotionController, but it calculates the center
 * and steer commands in a slightly more elegant method. <br> <a href=https://en.wikipedia.org/wiki/B%C3%A9zier_curve>Paper
 * with some of the math (Theorem 8.5.2)
 * </a> <br>
 * The rest of the math was from Team 254's 2018 Nonlinear Feedback functions.
 *
 * @author Russell Newton, Walton Robotics
 **/
public class RamseteController extends MotionController {

  private final boolean useMotorProfiles;
  private final boolean useDrivetrainSuppliedHeading;

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods
   * from
   * @param setSpeeds something implementing the SetSpeeds interface
   * @param useMotorProfiles whether or not you're paying much attention to motor profiles. Defaults
   * to false.
   * @param usingCamera whether or not you're using a camera. Defaults to false
   */
  public RamseteController(RobotConfig robotConfig, SetSpeeds setSpeeds, boolean useMotorProfiles,
      boolean useDrivetrainSuppliedHeading, Supplier<Boolean> usingCamera) {
    super(robotConfig, setSpeeds, usingCamera);
    this.useMotorProfiles = useMotorProfiles;
    this.useDrivetrainSuppliedHeading = useDrivetrainSuppliedHeading;
  }

  /**
   * @param robotConfig - the robotConfig to use the org.waltonrobotics.AbstractDrivetrain methods
   * from
   * @param setSpeeds something implementing the SetSpeeds interface
   * @param usingCamera whether or not you're using a camera. Defaults to false
   */
  public RamseteController(RobotConfig robotConfig, SetSpeeds setSpeeds,
      Supplier<Boolean> usingCamera) {
    this(robotConfig, setSpeeds, false, false, usingCamera);
  }

  //sinc(x) = sin(x) / x
  private static double sinc(double theta) {
    return theta != 0 ? Math.sin(theta) / theta : 1;
  }

  /**
   * Based on Team 254's 2018 code.
   */
  @Override
  public RobotPair findSpeeds(double time) {
    double leftVelocity = targetPathData.getLeftState().getVelocity();
    double rightVelocity = targetPathData.getRightState().getVelocity();
    double targetVelocity = (leftVelocity + rightVelocity) / 2;
    double targetOmega = (rightVelocity - leftVelocity) * (Path.getRobotWidth() / 2);
    double dynamicConstant = dynamicConstant(targetVelocity, targetOmega);

    //v = (v_d)(cos(dtheta)) + k(v_d, w_d)(lag)
    //w = w_d + (b)(v_d)sinc(dtheta)(xtrack) + k(v_d, w_d)(dtheta)
    double velocityCommand =
        targetVelocity * Math.cos(errorVector.getAngle()) + dynamicConstant * errorVector.getLag();
    double omegaCommand = targetOmega +
        robotConfig.getKBeta() * targetVelocity * sinc(errorVector.getAngle()) *
            errorVector.getXTrack() + dynamicConstant * errorVector.getAngle();
//    System.out.println(velocityCommand + " " + omegaCommand);

    //And here starts the help from 254
    double leftCommand = (velocityCommand - omegaCommand * robotConfig.effectiveWheelbaseRadius())
        / robotConfig.wheelRadius();
    double rightCommand =
        (velocityCommand + omegaCommand * robotConfig.effectiveWheelbaseRadius())
            / robotConfig.wheelRadius();
//    System.out.println(leftCommand + " " + rightCommand);

    double leftVoltage;
    double rightVoltage;
    if (useMotorProfiles) {

      double leftAcceleration = targetPathData.getLeftState().getAcceleration();
      double rightAcceleration = targetPathData.getRightState().getAcceleration();
      double centerAcceleration = (leftAcceleration + rightAcceleration) / 2;
      double centerAlpha = (rightAcceleration - leftAcceleration) * (Path.getRobotWidth() / 2);

      //Calculating torques
      double leftTorque = robotConfig.wheelRadius() / 2 *
          (centerAcceleration * robotConfig.robotMass() -
              centerAlpha * robotConfig.robotMOI() / robotConfig.effectiveWheelbaseRadius() -
              omegaCommand * robotConfig.robotAngularDrag() / robotConfig
                  .effectiveWheelbaseRadius());
      double rightTorque = robotConfig.wheelRadius() / 2 *
          (centerAcceleration * robotConfig.robotMass() +
              centerAlpha * robotConfig.robotMOI() / robotConfig.effectiveWheelbaseRadius() +
              omegaCommand * robotConfig.robotAngularDrag() / robotConfig
                  .effectiveWheelbaseRadius());

      leftVoltage = robotConfig.leftMotorConfig().getMotorParameters()
          .getVoltageFromTorque(leftCommand,
              leftTorque);
      rightVoltage = robotConfig.rightMotorConfig().getMotorParameters()
          .getVoltageFromTorque(rightCommand,
              rightTorque);
    } else {
      leftVoltage = robotConfig.leftMotorConfig().getMotorParameters()
          .getVoltageFromSpeed(leftCommand);
      rightVoltage = robotConfig.rightMotorConfig().getMotorParameters()
          .getVoltageFromSpeed(rightCommand);
    }
    return new RobotPair(leftVoltage, rightVoltage, time);
  }

  //k(v_d, w_d) = 2(z)(sqrt(w_d^2 + (b)(v_d^2))
  private double dynamicConstant(double targetVelocity, double targetOmega) {
    return 2 * robotConfig.getKZeta() * Math.sqrt(robotConfig.getKBeta() * Math.pow(targetVelocity,
        2) + Math.pow(targetOmega, 2));
  }

  @Override
  public Pose updateActualPosition(RobotPair wheelPositions, RobotPair previousWheelPositions,
      Pose estimatedActualPosition) {
    if (!useDrivetrainSuppliedHeading) {
      return super
          .updateActualPosition(wheelPositions, previousWheelPositions, estimatedActualPosition);
    } else {
      return super.updateActualPosition(wheelPositions, previousWheelPositions,
          estimatedActualPosition, setSpeeds.getSensorCalculatedHeading(), null);
    }
  }
}
