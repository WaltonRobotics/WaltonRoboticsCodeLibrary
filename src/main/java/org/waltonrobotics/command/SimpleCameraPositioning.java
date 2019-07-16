package org.waltonrobotics.command;

import org.waltonrobotics.metadata.CameraData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.motion.Spline;

public class SimpleCameraPositioning extends SimpleMotion {


  public SimpleCameraPositioning(Pose robotCameraEstimatedPosition, Pose targetPosition,
      double maxVelocity,
      double maxAcceleration,
      double startVelocity, double endVelocity, boolean isBackwards) {
    super(new Spline(
        maxVelocity,
        maxAcceleration,
        startVelocity,
        endVelocity,
        isBackwards,
        robotCameraEstimatedPosition,
        targetPosition
    ));

    SimpleMotion.getDrivetrain().setStartingPosition(robotCameraEstimatedPosition);
  }

  public SimpleCameraPositioning(CameraData cameraData, double maxVelocity, double maxAcceleration,
      double startVelocity, double endVelocity, boolean isBackwards) {
    this(cameraData.getCameraPose(), maxVelocity, maxAcceleration, startVelocity, endVelocity,
        isBackwards);
  }

  public SimpleCameraPositioning(Pose cameraData, double maxVelocity, double maxAcceleration,
      double startVelocity, double endVelocity, boolean isBackwards) {
    this(cameraData, Pose.ZERO.offset(-getDrivetrain().getRobotConfig().getRobotLength() / 2.0),
        maxVelocity,
        maxAcceleration, startVelocity,
        endVelocity, isBackwards);
  }

  public static SimpleCameraPositioning toCameraTarget(CameraData cameraData) {
    return new SimpleCameraPositioning(cameraData,
        getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(), 0, 0, false);
  }

  public static SimpleCameraPositioning toCameraTarget(Pose estimatedRobotPosition) {
    return new SimpleCameraPositioning(estimatedRobotPosition,
        getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(), 0, 0, false);
  }

  public static SimpleCameraPositioning toCameraTarget(Pose estimatedRobotPosition,
      Pose targetPosition) {
    return new SimpleCameraPositioning(estimatedRobotPosition, targetPosition,
        getDrivetrain().getRobotConfig().getMaxVelocity(),
        getDrivetrain().getRobotConfig().getMaxAcceleration(), 0, 0, false);
  }
}
