package org.waltonrobotics;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.motion.curve.BezierCurve;
import org.waltonrobotics.motion.curve.Spline;

public class TestCurves {

  private static int steps = 100;
  private static Point a = new Point(0, 0);
  private static Point b = new Point(2, 5);
  private static Point c = new Point(5, 2);
  private static Point d = new Point(10, 10);

  /**
   * Run this class to test outputs of Spline and Bezier Curve
   */
  public static void main(String[] args) {
    System.out.println("Bezier Curve with control points a, b, c, and d");
    BezierCurve curve = new BezierCurve(steps, a, b, c, d);
    for (Point point : curve.getPathPoints()) {
      locatePoints(point.getX(), point.getY());
    }
    System.out.println("Spline with knots a, b, c, and d");
    Spline spline = new Spline(steps, a, b, c, d);
    Point[] centerPoints = spline.getPathPoints();
    Point[] leftPoints = spline.getLeftPath();
    Point[] rightPoints = spline.getRightPath();
    double[] dts = spline.getDTsOnPath();
    for (int i = 0; i < leftPoints.length; i++) {
      locateSidePoints(leftPoints[i].getX(), leftPoints[i].getY(), centerPoints[i].getX(),
          centerPoints[i].getY(),
          rightPoints[i].getX(), rightPoints[i].getY(), dts[i]);
    }
  }

  private static void locateSidePoints(double xL, double yL, double xC, double yC, double xR,
      double yR, double dt) {
    System.out.printf(
        "xL: %01.03f \t yL: %01.03f \t xC: %01.03f \t yC: %01.03f \t xR: %01.03f \t yR: %01.03f \t dt: %f \n",
        xL, yL, xC, yC, xR, yR, dt);
  }

  private static void locatePoints(double x, double y) {
    System.out.printf("x: %01.03f \t y: %01.03f \n", x, y);
  }
}
