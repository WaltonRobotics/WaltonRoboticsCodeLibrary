package org.waltonrobotics.command.autolearning;

import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.command.Command;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;

//TODO test this
public class LearnPath extends Command {

  private static int index = 0;
  private final int useLast;
  private final String fileName;
  private final List<PathData> moments;
  private final NetworkButton stopButton;
  private final String buttonName;
  private final boolean useButtonToStop;
  private double movementTolerance = .1; // This is in meters
  private double angleTolerance = 15.0;
  private double maxVelocity;
  private double maxAcceleration;

  public LearnPath(int useLast, String fileName, boolean useButtonToStop) {

    this.useLast = useLast;
    this.useButtonToStop = useButtonToStop;
    moments = new LinkedList<>();
    this.fileName = fileName;

    index++;

    if (useButtonToStop) {
      //TODO check if you can just use stopButton.getName() instead of saving value into variable
      stopButton = new NetworkButton("SmartDashboard",
          buttonName = String.format("Stop %s %d", getClass().getName(), index)
      );
    } else {
      stopButton = null;
      buttonName = null;
    }
  }

  public LearnPath(int useLast, boolean useButtonToStop) {
    this(useLast, null, useButtonToStop);
  }

  public LearnPath(int useLast) {
    this(useLast, null, true);
  }


  @Override
  protected void end() {
    super.end();
    stopButton.free(); // TODO test if this deletes the button
    SimpleMotion.getDrivetrain().setSpeeds(0, 0); //Stops the robot from moving

//		NetworkTableInstance.getDefault().getTable("SmartDashboard").delete(buttonName); // try this if it does not work
//		System.out.println(stopButton.getName());
//		NetworkTableInstance.getDefault().getTable("SmartDashboard").delete(stopButton.getName());
  }

  @Override
  protected void initialize() {

    moments.clear();

  }

  @Override
  protected void execute() {
//		Gets the robots current state (velocity , etc..)
    PathData instance = SimpleMotion.getDrivetrain().getCurrentRobotState();

//		Looks if there is a new max velocity
    maxVelocity = Math.max(
        Math.max(
            Math.abs(instance.getLeftState().getVelocity()),
            Math.abs(instance.getRightState().getVelocity())
        ),
        maxVelocity);

//		Looks if there is a new max acceleration
    maxAcceleration = Math.max(
        Math.max(
            Math.abs(instance.getLeftState().getAcceleration()),
            Math.abs(instance.getRightState().getAcceleration())
        ),
        maxAcceleration);

//		Adds the instance to a list
    moments.add(instance);
  }


  @Override
  protected boolean isFinished() {

    if (useButtonToStop) {
      return stopButton.get(); // Pressing the stop button on SmartDashboard will stop the program
    } else {
//		Checks how many you last moments you should use to average
      if (moments.size() >= useLast) {

        double movementChange = 0;
        double angleChange = 0;

        int size = moments.size();

//			Loops though the last useLast moments of the list
        for (int i = size - (useLast - 1) - 1; i < size; i++) {
//				Retrieves the current and last moments
          Pose currentPosition = moments.get(i).getCenterPose();
          Pose previousPosition = moments.get(i - 1).getCenterPose();

//				Gets the absolute difference of the xs, ys and angles
          double dX = Math.abs(currentPosition.getX() - previousPosition.getX());
          double dY = Math.abs(currentPosition.getY() - previousPosition.getY());
          double dAngle = Math.abs(currentPosition.getAngle() - previousPosition.getAngle());

          movementChange += StrictMath.hypot(dX, dY);
          angleChange += dAngle;
        }

        // If there is less than or equal to a 10 cm change and there is less than or equal to 15 degrees of change
        return (movementChange <= movementTolerance) &&
            (angleChange <= StrictMath.toRadians(angleTolerance));
      }
    }
    return false;
  }

  public List<PathData> getMoments() {
    return moments;
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  public String getFileName() {
    return fileName;
  }

  public int getUseLast() {
    return useLast;
  }

  public double getMovementTolerance() {
    return movementTolerance;
  }

  public void setMovementTolerance(double movementTolerance) {
    this.movementTolerance = movementTolerance;
  }

  public double getAngleTolerance() {
    return angleTolerance;
  }

  public void setAngleTolerance(double angleTolerance) {
    this.angleTolerance = angleTolerance;
  }

  @Override
  public String toString() {
    return "LearnPath{" +
        "useLast=" + useLast +
        ", fileName='" + fileName + '\'' +
        ", moments=" + moments +
        '}';
  }


}
