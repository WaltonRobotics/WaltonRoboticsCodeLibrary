package org.waltonrobotics.command.autolearning;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.motion.Path;
import org.waltonrobotics.motion.SavePath;

public class PathLearner extends Command {

  private static final double VELOCITY_TOLERANCE = .1;
  private final String fileName = String
      .format("Learned Path %s", new SimpleDateFormat("yyyy-MM-dd hh-mm-ss").format(new Date()));

  private final boolean willSaveWhenFinished;
  private final int lastUse;
  private final JoystickButton joystickButton;

  private final LinkedList<PathData> movement = new LinkedList<>();

  private double averageVelocity = 0;
  private Pose startPosition;

  private double maxVelocity;
  private double maxAcceleration;

  private Path motion;

  public PathLearner(int lastUse, boolean willSaveWhenFinished) {
    this.lastUse = lastUse;
    this.willSaveWhenFinished = willSaveWhenFinished;
    joystickButton = null;
  }


  public PathLearner(int lastUse) {
    this(lastUse, true);
  }

  public PathLearner(JoystickButton joystickButton, boolean willSaveWhenFinished) {
    this.joystickButton = joystickButton;
    this.willSaveWhenFinished = willSaveWhenFinished;
    lastUse = -1;
  }

  public PathLearner(JoystickButton stopButton) {
    this(stopButton, true);
  }

  public static double getVelocityTolerance() {
    return VELOCITY_TOLERANCE;
  }

  @Override
  protected void initialize() {
    startPosition = SimpleMotion.getDrivetrain().getActualPosition();
  }

  public PathData offset(PathData current) {
    Pose pose = current.getCenterPose()
        .offset(startPosition.getX(), startPosition.getY(), startPosition.getAngle());

    return new PathData(current.getLeftState(), current.getRightState(), pose, current.getTime(),
        current.isBackwards());
  }

  @Override
  protected void execute() {
    PathData currentRobotState = SimpleMotion.getDrivetrain().getCurrentRobotState();
    currentRobotState = offset(currentRobotState);

    movement.add(currentRobotState);

    averageVelocity +=
        (Math.abs(currentRobotState.getLeftState().getVelocity()) + Math
            .abs(currentRobotState.getLeftState().getVelocity())) / (2 * lastUse);

    maxVelocity = Math.max(
        Math.max(
            Math.abs(currentRobotState.getLeftState().getVelocity()),
            Math.abs(currentRobotState.getRightState().getVelocity())
        ),
        maxVelocity);

//		Looks if there is a new max acceleration
    maxAcceleration = Math.max(
        Math.max(
            Math.abs(currentRobotState.getLeftState().getAcceleration()),
            Math.abs(currentRobotState.getRightState().getAcceleration())
        ),
        maxAcceleration);

    if (movement.size() > lastUse) {
      PathData pathData = movement.get(movement.size() - 1 - lastUse);

      averageVelocity -= (Math.abs(pathData.getLeftState().getVelocity()) + Math
          .abs(pathData.getLeftState().getVelocity())) / (2 * lastUse);
    }
  }

  @Override
  protected boolean isFinished() {
    return joystickButton == null ? averageVelocity <= VELOCITY_TOLERANCE : joystickButton.get();
  }

  @Override
  protected void end() {
    motion = SavePath.createPathData(maxVelocity, maxAcceleration, movement);
    if (willSaveWhenFinished) {
      motion.savePath(fileName);
    }
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  public Path getMotion() {
    return motion;
  }

  public String getFileName() {
    return fileName;
  }

  public boolean isWillSaveWhenFinished() {
    return willSaveWhenFinished;
  }

  public int getLastUse() {
    return lastUse;
  }

  public JoystickButton getJoystickButton() {
    return joystickButton;
  }

  public LinkedList<PathData> getMovement() {
    return movement;
  }

  public double getAverageVelocity() {
    return averageVelocity;
  }

  public Pose getStartPosition() {
    return startPosition;
  }
}
