package org.waltonrobotics.metadata;

/**
 * This holds the x, y, and angle of the robot.
 *
 * @author Russell Newton, Walton Robotics
 */
public class Pose {

  public static final Pose ZERO = new Pose();

  private final double x;
  private final double y;
  private final double angle;

  /**
   * @param x the x coordinate in space
   * @param y the y coordinate in space
   * @param angle the angle in degrees
   */
  public Pose(double x, double y, double angle) {
    this.x = x;
    this.y = y;
    this.angle = angle;
  }

  /**
   * Creates a Pose without specifying an angle
   */
  public Pose(double x, double y) {
    this(x, y, 0);
  }

  /**
   * Creates a Pose at 0,0 facing 0 degrees
   */
  public Pose() {
    this(0, 0, 0);
  }


  /**
   * @return the x value of the point
   */
  public final double getX() {
    return x;
  }

  /**
   * @return the y value of the point
   */
  public final double getY() {
    return y;
  }

  /**
   * @return the angle of the point in radians
   */
  public final double getAngle() {
    return angle;
  }

  /**
   * @return the angle of the point in degrees
   */
  public final double getDegrees() {
    return StrictMath.toDegrees(angle);
  }

  /**
   * Finds the distance between two points
   *
   * @param otherPoint the point you want to find the distance from
   * @return the distance from this point to the other point
   */
  public final double distance(Pose otherPoint) {
    return StrictMath.sqrt(
        StrictMath.pow(x - otherPoint.x, 2.0) + StrictMath
            .pow(y - otherPoint.y, 2.0));
  }

  /**
   * Rotates a point around a central point. Imagine making an arc on a circle
   *
   * @param centerPoint the center of the circle
   * @param arcAngle the angle to rotate the point to (degrees)
   * @param backwards whether or not to rotate the point backwards (clockwise)
   * @return the rotated point
   */
  public final Pose rotate(Pose centerPoint, double arcAngle, boolean backwards, double scale) {
    double distance = distance(centerPoint) * (backwards ? -1 : 1) * scale;
    double yDisplacement = distance * StrictMath.sin(arcAngle);
    double xDisplacement = distance * StrictMath.cos(arcAngle);
    return new Pose(centerPoint.x + xDisplacement, centerPoint.y + yDisplacement);
  }

  /**
   * Checks if a another point has the same x or y as another.
   *
   * @param other the point to compare to
   * @return true if the other point has the same coordinate as the the point instance, false
   * otherwise
   */
  public final boolean sameCoordinates(Pose other) {
    return (other.getX() == getX()) && (other.getY() == getY());
  }

  /**
   * Creates an offset Pose by a dX, dY, and dAngle
   *
   * @return the offset Pose
   */
  public final Pose offset(double dX, double dY, double dAngle) {
    return new Pose(x + dX, y + dY, angle + dAngle);
  }

  public final Pose offset(double distance) {
    return new Pose((StrictMath.cos(getAngle()) * distance) + getX(),
        (StrictMath.sin(getAngle()) * distance) + getY(),
        getAngle());
  }

  public Pose multiply(double constant) {
    return new Pose(x * constant, y * constant, angle);
  }

  public Pose add(Pose add) {
    return offset(add.x, add.y, 0);
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }

    Pose pose = (Pose) o;

    if (Double.compare(pose.x, x) != 0) {
      return false;
    }
    if (Double.compare(pose.y, y) != 0) {
      return false;
    }
    return Double.compare(pose.angle, angle) == 0;

  }

  @Override
  public int hashCode() {
    int result;
    long temp;
    temp = Double.doubleToLongBits(x);
    result = (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(y);
    result = 31 * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(angle);
    result = 31 * result + (int) (temp ^ (temp >>> 32));
    return result;
  }

  @Override
  public String toString() {

//    return String.format("Pose{x=%.3f, y=%.3f, angle=%f}", x,y,angle);
    return "Pose{" +
        "x=" + x +
        ", y=" + y +
        ", angle=" + angle +
        '}';
  }
}
