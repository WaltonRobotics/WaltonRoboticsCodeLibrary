package org.waltonrobotics.metadata;

public class CameraData {

  private final Pose cameraPose;
  private final double height;
  private final int numberOfTargets;
  private final double time;

  public CameraData(double x, double y, double height, double angle, int numberOfTargets,
      double time) {
    cameraPose = new Pose(x, y, StrictMath.toRadians(angle));
    this.height = height;
    this.numberOfTargets = numberOfTargets;
    this.time = time;
  }

  public CameraData(int numberOfTargets) {
    this(0, 0, 0, 0, numberOfTargets, -1.0);
  }

  public CameraData() {
    this(0, 0, 0, 0, 0, -1.0);
  }

  public Pose getCameraPose() {
    return cameraPose;
  }

  public double getHeight() {
    return height;
  }

  public int getNumberOfTargets() {
    return numberOfTargets;
  }

  public double getTime() {
    return time;
  }

  @Override
  public String toString() {
    return "CameraData{" +
        "cameraPose=" + cameraPose +
        ", height=" + height +
        ", numberOfTargets=" + numberOfTargets +
        ", time=" + time +
        '}';
  }
}
