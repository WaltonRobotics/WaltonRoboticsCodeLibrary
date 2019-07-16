package org.waltonrobotics.command;

import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.motion.Line;

public class SimpleLine extends SimpleMotion {

  public SimpleLine(double maxVelocity, double maxAcceleration, double startVelocity,
      double endVelocity,
      boolean isBackwards, Pose startPosition, Pose endPosition) {
    super(new Line(maxVelocity, maxAcceleration, startVelocity, endVelocity, isBackwards,
        startPosition,
        endPosition));
  }

  public static SimpleLine lineWithDistance(Pose startPosition, double distance) {
    return lineWithDistance(getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(),
        startPosition,
        distance);
  }


  public static SimpleLine lineWithDistance(double distance) {
    return lineWithDistance(getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(),
        getDrivetrain().getActualPosition(),
        distance);
  }

  public static SimpleLine lineWithDistance(boolean isBackwards, Pose startPosition,
      double distance) {
    return new SimpleLine(getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(), 0,
        0, isBackwards,
        startPosition,
        startPosition.offset(distance));
  }

  public static SimpleLine lineWithDistance(boolean isBackwards,
      double distance) {
    return new SimpleLine(getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(), 0,
        0, isBackwards,
        getDrivetrain().getActualPosition(),
        getDrivetrain().getActualPosition().offset(distance));
  }

  public static SimpleLine lineWithDistance(double maxVelocity, double maxAcceleration,
      Pose startPosition,
      double distance) {
    return lineWithDistance(maxVelocity, maxAcceleration, false, startPosition, distance);
  }


  public static SimpleLine lineWithDistance(double maxVelocity, double maxAcceleration,
      double distance) {
    return lineWithDistance(maxVelocity, maxAcceleration, false,
        getDrivetrain().getActualPosition(), distance);
  }

  public static SimpleLine lineWithDistance(double maxVelocity, double maxAcceleration,
      boolean isBackwards,
      Pose startPosition,
      double distance) {
    return new SimpleLine(maxVelocity, maxAcceleration, 0, 0, isBackwards, startPosition,
        startPosition.offset(distance));
  }

  public static SimpleLine lineWithDistance(double maxVelocity, double maxAcceleration,
      boolean isBackwards,

      double distance) {
    return new SimpleLine(maxVelocity, maxAcceleration, 0, 0, isBackwards,
        getDrivetrain().getActualPosition(),
        getDrivetrain().getActualPosition().offset(distance));
  }
}
