package org.waltonrobotics.motion;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.State;

/**
 * Extend this if you want to make your own Motion.
 *
 * @author Russell Newton, Walton Robotics
 */
public abstract class Path {

  //FIXME 1000 points per meter?
  public static int pathNumberOfSteps = 1000; // TODO find better name for this variable. Also before it was 50 but maybe try smart
  private static double robotWidth; // WHat if you have multiple robots running the same code? Should we account for that scenario?
  private final boolean isBackwards;
  private final LinkedList<PathData> pathData;
  protected double vCruise;
  protected double aMax;
  protected List<Pose> keyPoints;
  private boolean isFinished;


  /**
   * @param vCruise cruise velocity of the robot, the velocity that the robot should try to reach
   * @param aMax the maximum acceleration the robot should achieve
   * @param isBackwards if the robot is travelling forwards or backwards
   * @param keyPoints the points that define the path
   */
  protected Path(double vCruise, double aMax, boolean isBackwards, List<Pose> keyPoints) {
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

  public Path(double vCruise, double aMax, boolean isBackwards, Pose... keyPoints) {
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
    Path.robotWidth = robotWidth;
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

  public static Path loadPath(String filePath) throws IOException {
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
      Path path = new Path(maxVelocity[0], maxAcceleration[0], isBackwards[0], keyPoints) {
      };

      path.getPathData().clear();
      path.getPathData().addAll(pathDataList);

      return path;
    }
  }

  public static Path loadingPathFromString(String pathAsString)
      throws ClassNotFoundException, NoSuchMethodException, IllegalAccessException, InvocationTargetException, InstantiationException {
    return loadingPathFromString(pathAsString, 1.0);
  }

  public static Path loadingPathFromString(String pathAsString, double scale)
      throws ClassNotFoundException, NoSuchMethodException, IllegalAccessException, InvocationTargetException, InstantiationException {
    String[] data = pathAsString.split("\\s");

    int i = 0;

    String className = data[i++];

    List<Class<?>> classTypes = new ArrayList<>();
    List<Object> typesValues = new ArrayList<>();

    for (; i < data.length; i++) {
      String parameter = data[i];

      if ("PF".equals(parameter)) {
        i++;
        break;
      }

      ClassValuePair variable = loadParameter(parameter);

      classTypes.add(variable.getClassType());
      typesValues.add(variable.getValue());
    }

    if (i != data.length) {
      List<Pose> poseList = new LinkedList<>();

      for (; i < data.length; i += 3) {
        double x = Double.parseDouble(data[i]) / scale;
        double y = Double.parseDouble(data[i + 1]) / scale;
        double angle = Double.parseDouble(data[i + 2]);

        poseList.add(new Pose(x, y, angle));
      }

      classTypes.add(List.class);
      typesValues.add(poseList);
    }

    System.out.println(classTypes);

    Class<?> clazz = Class.forName(className);

    Constructor<?> ctor = clazz.getConstructor(classTypes.toArray(new Class<?>[0]));

    return (Path) ctor.newInstance(typesValues.toArray());
  }

  private static ClassValuePair loadParameter(String parameter) {

    ClassValuePair classObjectEntry = new ClassValuePair(String.class, parameter);

    try {
      Double integer = Double.parseDouble(parameter);

      classObjectEntry.setValue(integer);
      classObjectEntry.setClassType(Double.TYPE);
    } catch (NumberFormatException e1) {
      try {
        String[] parameters = parameter.split(",");

        if (parameters.length == 3) {
          double x = Double.parseDouble(parameters[0]);
          double y = Double.parseDouble(parameters[1]);
          double angle = Double.parseDouble(parameters[2]);

          classObjectEntry.setClassType(Pose.class);
          classObjectEntry.setValue(new Pose(x, y, angle));
        } else {
          Boolean integer = Boolean.parseBoolean(parameter);

          classObjectEntry.setClassType(Boolean.TYPE);
          classObjectEntry.setValue(integer);

        }
      } catch (NumberFormatException ignored) {

      }
    }

    return classObjectEntry;
  }

  protected void addKeyPoints(StringBuilder stringBuilder) {
    stringBuilder.append("PF"); // Parameters finished now moving to key points
    stringBuilder.append(' ');

    for (Pose keyPoints : getKeyPoints()) {
      double x = keyPoints.getX();
      double y = keyPoints.getY();
      double angle = keyPoints.getAngle();

      stringBuilder.append(x);
      stringBuilder.append(' ');
      stringBuilder.append(y);
      stringBuilder.append(' ');
      stringBuilder.append(angle);
      stringBuilder.append(' ');
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

  public void setKeyPoints(Collection<Pose> keyPoints) {

    this.keyPoints.clear();
    this.keyPoints.addAll(keyPoints);

  }

  public void setKeyPoints(Pose... keyPoints) {
    this.keyPoints.clear();
    Collections.addAll(this.keyPoints, keyPoints);

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

  public void savePath(String fileName) {
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

  public String convertToString() {
			/*
		double vCruise,
		double aMax,
		double startVelocity,
		double endVelocity,
		boolean isBackwards,
		List<Pose> controlPoints
		 */

    StringBuilder stringBuilder = new StringBuilder();

    addDefaultParameters(stringBuilder);
    addKeyPoints(stringBuilder);

    return stringBuilder.toString();
  }

  public void addDefaultParameters(StringBuilder stringBuilder) {
    String className = getClass().getName();

    stringBuilder.append(className);
    stringBuilder.append(' ');
    stringBuilder.append(getVCruise());
    stringBuilder.append(' ');
    stringBuilder.append(getAMax());
    stringBuilder.append(' ');
    stringBuilder.append(isBackwards());
    stringBuilder.append(' ');
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

  private static class ClassValuePair {

    private Class<?> classType;
    private Object value;

    public ClassValuePair(Class<?> classType, Object value) {
      this.classType = classType;
      this.value = value;
    }

    public Class<?> getClassType() {
      return classType;
    }

    public void setClassType(Class<?> classType) {
      this.classType = classType;
    }

    public Object getValue() {
      return value;
    }

    public void setValue(Object value) {
      this.value = value;
    }
  }
}
