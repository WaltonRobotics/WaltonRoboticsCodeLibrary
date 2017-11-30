package org.waltonrobotics.controller;

public class Point {

  private final double x;
  private final double y;
  private final double headingAngle;

  public Point(double x, double y, double headingAngle) {
    this.x = x;
    this.y = y;
    this.headingAngle = headingAngle;
  }

  public Point(double x, double y) {
    this(x, y, 0);
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getHeadingAngle() {
    return headingAngle;
  }

  public Point offsetPerpendicular(double dxAtPoint, double distance) {
    double angleOfDX = Math.atan(dxAtPoint);
    double offsetX = distance * Math
        .cos(angleOfDX + Math.PI / 2);  //Finds point at distance along perpendicular line
    double offsetY = distance * Math.sin(angleOfDX + Math.PI / 2);

    return new Point(this.x + offsetX, this.y + offsetY, angleOfDX);
  }

}
