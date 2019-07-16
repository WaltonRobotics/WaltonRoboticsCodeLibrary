package org.waltonrobotics.dynamicMotion;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.State;
import org.waltonrobotics.motion.Path;

/**
 * Extend this if you want to make your own Motion.
 *
 * @author Russell Newton, Walton Robotics
 */
public abstract class DynamicPath {

  //FIXME 1000 points per meter?
  public static int pathNumberOfSteps = 1000; // TODO find better name for this variable. Also before it was 50 but maybe try smart
  private static double robotWidth; // WHat if you have multiple robots running the same code? Should we account for that scenario?
  private final boolean isBackwards;
  private final List<Pose> keyPoints;
  private final LinkedList<PathData> pathData;
  protected double vCruise;
  protected double aMax;
  private boolean isFinished;

  /**
   * @param vCruise cruise velocity of the robot, the velocity that the robot should try to reach
   * @param aMax the maximum acceleration the robot should achieve
   * @param isBackwards if the robot is travelling forwards or backwards
   * @param keyPoints the points that define the path
   */
  protected DynamicPath(double vCruise, double aMax, boolean isBackwards, List<Pose> keyPoints) {
    this.isBackwards = isBackwards;
    this.keyPoints = keyPoints;
    if (vCruise == 0) {
      throw new IllegalArgumentException("vCruise cannot be 0");
    }
    this.vCruise = vCruise;

    if (aMax == 0) {
      throw new IllegalArgumentException("aMax cannot be 0");
    }
    this.aMax = aMax;
    isFinished = false;
    pathData = new LinkedList<>();
  }

  public DynamicPath(double vCruise, double aMax, boolean isBackwards, Pose... keyPoints) {
    this(vCruise, aMax, isBackwards, Arrays.asList(keyPoints));
  }

  /**
   * @return the number of steps the path should be divided into. Default is 50.
   */
  public static int getPathNumberOfSteps() {
    return pathNumberOfSteps;
  }

  /**
   * @param pathNumberOfSteps the new number of steps a path should be divided into
   */
  public static void setPathNumberOfSteps(int pathNumberOfSteps) {
    Path.pathNumberOfSteps = pathNumberOfSteps;
  }

  /**
   * @return The width of the robot from the outside of each wheel
   */
  public static double getRobotWidth() {
    return robotWidth;
  }

  /**
   * @param robotWidth the new width of the robot
   */
  public static void setRobotWidth(double robotWidth) {
    DynamicPath.robotWidth = robotWidth;
  }

  /**
   * Bounds an angle to be in between -PI and PI. if the angles are more or less then the angle will
   * cycle.
   *
   * @param angle angle to be bounded
   * @return the angle bounded
   */
  public static double boundAngle(double angle) {
    if (angle > Math.PI) {
      return angle - (2.0 * Math.PI);
    } else if (angle < -Math.PI) {
      return angle + (2.0 * Math.PI);
    }
    return angle;
  }


  public static DynamicPath loadPath(String filePath) throws IOException {
    try (BufferedReader bufferedReader = new BufferedReader(
        new InputStreamReader(new FileInputStream(filePath), StandardCharsets.UTF_8))) {

      List<PathData> pathDataList = new LinkedList<>();
      List<Pose> keyPoints = new LinkedList<>();

      final double[] pathDataSize = {1.0};
      final double[] keyPointSize = {1.0};

      final double[] maxVelocity = new double[1];
      final double[] maxAcceleration = new double[1];
      final boolean[] isBackwards = new boolean[1];
      final double[] robotWidth = new double[1];

      final int[] i = {0};

      bufferedReader.lines().forEach(line -> {
        double[] data = Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();

        if (i[0] == 1) {
          pathDataSize[0] = data[data.length - 6];
          keyPointSize[0] = data[data.length - 5];
          maxVelocity[0] = data[data.length - 4];
          maxAcceleration[0] = data[data.length - 3];
          isBackwards[0] = data[data.length - 2] == 1.0;
          robotWidth[0] = data[data.length - 1];
        }

        int index = i[0] - 1;

        if (index < pathDataSize[0]) {
          pathDataList.add(new PathData(
              new State(data[0], data[1], data[2]),
              new State(data[3], data[4], data[5]),
              new Pose(data[6], data[7], data[8]),
              data[9]
          ));
        }

        if (index < keyPointSize[0]) {
          keyPoints.add(new Pose(data[10], data[11], data[12]));
        }

        i[0]++;
      });

      Path.setRobotWidth(robotWidth[0]);
      DynamicPath path = new DynamicPath(maxVelocity[0], maxAcceleration[0], isBackwards[0],
          keyPoints) {
        @Override
        public PathData createPathData(PathData previousPathData, double percentage) {
          return null;
        }
      };

      path.getPathData().clear();
      path.getPathData().addAll(pathDataList);

      return path;
    }
  }

  /**
   * @return the path data for the whole path
   * @see PathData
   */
  public LinkedList<PathData> getPathData() {
    return pathData;
  }

  /**
   * @return if the robot is travelling backwards or not
   */
  public final boolean isBackwards() {
    return isBackwards;
  }

  /**
   * @return the key points that define the path
   */
  public final List<Pose> getKeyPoints() {
    return keyPoints;
  }

  public void setKeyPoints(Pose... keyPoints) {
    this.keyPoints.clear();
    Collections.addAll(this.keyPoints, keyPoints);

  }

  public void setKeyPoints(Collection<Pose> keyPoints) {

    this.keyPoints.clear();
    this.keyPoints.addAll(keyPoints);

  }

  /**
   * @return if the path has been completed
   */
  public final boolean isFinished() {
    return isFinished;
  }

  public final void setFinished(boolean finished) {
    isFinished = finished;
  }

  /**
   * @return the maximum acceleration the robot should be at
   */
  public final double getAMax() {
    return aMax;
  }

  /**
   * @return the velocity the robot should try to reach
   */
  public final double getVCruise() {
    return vCruise;
  }

  public abstract PathData createPathData(PathData previousPathData,
      double percentage);

  private void savePath(String fileName) {
    if (!getPathData().isEmpty() && (fileName != null)) {

      double maxSize = Math.max(Math.max(getPathData().size(), getKeyPoints().size()), 1);

      StringBuilder stringBuilder = new StringBuilder(getPathData().size() * 13 * 2);

      stringBuilder.append("Left Length,");
      stringBuilder.append("Left Velocity,");
      stringBuilder.append("Left Acceleration,");

      stringBuilder.append("Right Length,");
      stringBuilder.append("Right Velocity,");
      stringBuilder.append("Right Acceleration,");

      stringBuilder.append("Center X,");
      stringBuilder.append("Center Y,");
      stringBuilder.append("Center Angle,");

      stringBuilder.append("Time,");

      stringBuilder.append("Keypoint X,");
      stringBuilder.append("Keypoint Y,");
      stringBuilder.append("Keypoint Angle,");

      stringBuilder.append("Number of Path Data,");
      stringBuilder.append("Number of Keypoints,");

      stringBuilder.append("Velocity Cruise,");
      stringBuilder.append("Max Acceleration,");
      stringBuilder.append("Backwards,");
      stringBuilder.append("Robot width");

      stringBuilder.append('\n');

      for (int i = 0; i < maxSize; i++) {

        if (i < getPathData().size()) {
          PathData moment = getPathData().get(i);
          stringBuilder.append(moment.getLeftState().getLength());
          stringBuilder.append(',');
          stringBuilder.append(moment.getLeftState().getVelocity());
          stringBuilder.append(',');
          stringBuilder.append(moment.getLeftState().getAcceleration());
          stringBuilder.append(',');

          stringBuilder.append(moment.getRightState().getLength());
          stringBuilder.append(',');
          stringBuilder.append(moment.getRightState().getVelocity());
          stringBuilder.append(',');
          stringBuilder.append(moment.getRightState().getAcceleration());
          stringBuilder.append(',');

          stringBuilder.append(moment.getCenterPose().getX());
          stringBuilder.append(',');
          stringBuilder.append(moment.getCenterPose().getY());
          stringBuilder.append(',');
          stringBuilder.append(moment.getCenterPose().getAngle());
          stringBuilder.append(',');

          stringBuilder.append(moment.getTime());
          stringBuilder.append(',');
        } else {
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
        }

        if (i < getKeyPoints().size()) {
          Pose keyPoint = getKeyPoints().get(i);
          stringBuilder.append(keyPoint.getX());
          stringBuilder.append(',');
          stringBuilder.append(keyPoint.getY());
          stringBuilder.append(',');
          stringBuilder.append(keyPoint.getAngle());
          stringBuilder.append(',');
        } else {
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
        }

        if (i == 0) {
          stringBuilder.append(getPathData().size());
          stringBuilder.append(',');
          stringBuilder.append(getKeyPoints().size());
          stringBuilder.append(',');

          stringBuilder.append(getVCruise());
          stringBuilder.append(',');
          stringBuilder.append(getAMax());
          stringBuilder.append(',');
          stringBuilder.append(isBackwards() ? 1 : 0);
          stringBuilder.append(',');
          stringBuilder.append(getRobotWidth());
        } else {
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
          stringBuilder.append(',');
        }

        stringBuilder.append('\n');
      }

      try (BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(fileName))) {
        bufferedWriter.write(stringBuilder.toString());
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  @Override
  public String toString() {
    return "Path{" +
        "vCruise=" + vCruise +
        ", aMax=" + aMax +
        ", isBackwards=" + isBackwards +
        ", keyPoints=" + keyPoints +
        ", pathData=" + pathData +
        ", isFinished=" + isFinished +
        '}';
  }
}
