package org.waltonrobotics.command;

import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.motion.PointTurn;

public class SimpleTurn extends SimpleMotion {

  public SimpleTurn(double maxVelocity, double maxAcceleration, Pose startPosition,
      double endAngle) {
    super(new PointTurn(
        maxVelocity,
        maxAcceleration,
        startPosition,
        endAngle));
  }

  public static SimpleTurn pointTurn(Pose startPosition, double endAngle) {
    return pointTurn(getDrivetrain().getRobotConfig().getMaxVelocity() / 4.0,
        getDrivetrain().getRobotConfig().getMaxAcceleration(),
        startPosition,
        endAngle);
  }

  public static SimpleTurn pointTurn(double endAngle) {
    return pointTurn(getDrivetrain().getRobotConfig().getMaxVelocity() / 4.0,
        getDrivetrain().getRobotConfig().getMaxAcceleration(),
        getDrivetrain().getActualPosition(),
        endAngle);
  }

  public static SimpleTurn pointTurn(double maxVelocity, double maxAcceleration,
      Pose startPosition, double endAngle) {
    return new SimpleTurn(maxVelocity, maxAcceleration, startPosition, endAngle);
  }


  public static SimpleTurn pointTurn(double maxVelocity, double maxAcceleration,
      double endAngle) {
    return new SimpleTurn(maxVelocity, maxAcceleration, getDrivetrain().getActualPosition(),
        endAngle);
  }

  public static SimpleTurn pointTurn(Pose startPosition, Pose endPosition) {
    return pointTurn(getDrivetrain().getRobotConfig().getMaxVelocity() / 2.0,
        getDrivetrain().getRobotConfig().getMaxAcceleration(),
        startPosition,
        endPosition.getAngle());
  }


  public static SimpleTurn pointTurn(Pose endPosition) {
    return pointTurn(getDrivetrain().getRobotConfig().getMaxVelocity() / 2.0,
        getDrivetrain().getRobotConfig().getMaxAcceleration(),
        getDrivetrain().getActualPosition(),
        endPosition.getAngle());
  }
}
