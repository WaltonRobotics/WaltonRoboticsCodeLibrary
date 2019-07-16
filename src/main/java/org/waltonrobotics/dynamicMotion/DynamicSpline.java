package org.waltonrobotics.dynamicMotion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.ListIterator;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.State;
import org.waltonrobotics.motion.BezierCurve;

public class DynamicSpline extends DynamicPath {

  private final double startAngle;
  private final double endAngle;
  private final double startScale;
  private final double endScale;
  private final double startVelocity;
  private final double endVelocity;
  private final double curveLength;
  private List<DynamicBezierCurve> bezierCurves;

  /**
   * Construct a spline. Note that the x axis is the direction the robot is facing if the start
   * angle is 0
   *
   * @param vCruise - max velocity
   * @param aMax - max acceleration
   * @param startVelocity - the starting velocity of the Path
   * @param endVelocity - the ending velocity of the Path
   * @param isBackwards - if the robot will be moving backwards, make this true
   * @param knots - the points you want the robot to drive through
   */
  public DynamicSpline(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards,
      double scaleStart, double scaleEnd,
      List<Pose> knots) {
    super(vCruise, aMax, isBackwards, knots);
    this.startAngle = knots.get(0).getAngle();
    this.endAngle = knots.get(knots.size() - 1).getAngle();
    startScale = scaleStart;
    endScale = scaleEnd;
    this.endVelocity = endVelocity;
    this.startVelocity = startVelocity;

    createPath();
    curveLength = bezierCurves.stream().mapToDouble(DynamicBezierCurve::getCurveLength).sum();
  }

  public DynamicSpline(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards, Pose... knots) {
    this(vCruise, aMax, startVelocity, endVelocity,
        isBackwards, 1.0, 1.0, Arrays.asList(knots));
  }

  public DynamicSpline(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards, List<Pose> knots) {
    this(vCruise, aMax, startVelocity, endVelocity,
        isBackwards, 1.0, 1.0, knots);
  }

  public PathData createPathData(double percentage) {
    return getPathData(percentage);
  }

  private PathData getPathData(double percentage) {
    DynamicBezierCurve dynamicBezierCurve = bezierCurves.get((int) percentage);

    dynamicBezierCurve.getPathData(percentage - ((int) percentage));

    return null;
  }

  public Pose getPoint(double percentage) {
    if (percentage != 1.0) {
      int value = (int) (percentage * bezierCurves.size());
      DynamicBezierCurve dynamicBezierCurve = bezierCurves
          .get(value);

      return dynamicBezierCurve
          .getPoint(((percentage * bezierCurves.size()) - value));
    }
    return bezierCurves.get(bezierCurves.size() - 1).getPoint(1.0);

  }

  public Pose getDerivative(double percentage) {
    if (percentage != 1.0) {
      int value = (int) (percentage * bezierCurves.size());
      DynamicBezierCurve dynamicBezierCurve = bezierCurves
          .get(value);

      return dynamicBezierCurve
          .getDerivative(((percentage * bezierCurves.size()) - value));
    }
    return bezierCurves.get(bezierCurves.size() - 1).getDerivative(1.0);
  }

  /**
   * Creates the control points required to make cubic bezier curves that transition between knots.
   * Will make them for the shortest path possible.
   *
   * @return A list of lists that hold the control points for the segments in the spline
   */
  private List<List<Pose>> computeControlPoints(List<Pose> knots) {
    int degree = knots.size() - 1;
    Pose[] points1 = new Pose[degree];
    Pose[] points2 = new Pose[degree];

    /* constants for Thomas Algorithm */
    double[] a = new double[degree];
    double[] b = new double[degree];
    double[] c = new double[degree];
    double[] rX = new double[degree];
    double[] rY = new double[degree];

    /* left most segment */
    a[0] = 0;
    b[0] = 2;
    c[0] = 1;
    rX[0] = knots.get(0).getX() + (2 * knots.get(1).getX());
    rY[0] = knots.get(0).getY() + (2 * knots.get(1).getY());

    /* internal segments */
    for (int i = 1; i < (degree - 1); i++) {
      a[i] = 1;
      b[i] = 4;
      c[i] = 1;
      rX[i] = (4 * knots.get(i).getX()) + (2 * knots.get(i + 1).getX());
      rY[i] = (4 * knots.get(i).getY()) + (2 * knots.get(i + 1).getY());
    }

    /* right segment */
    a[degree - 1] = 2;
    b[degree - 1] = 7;
    c[degree - 1] = 0;
    rX[degree - 1] = (8 * knots.get(degree - 1).getX()) + knots.get(degree).getX();
    rY[degree - 1] = (8 * knots.get(degree - 1).getY()) + knots.get(degree).getY();

    /* solves Ax=b with the Thomas algorithm */
    for (int i = 1; i < degree; i++) {
      double m = a[i] / b[i - 1]; // temporary variable
      b[i] -= m * c[i - 1];
      rX[i] -= m * rX[i - 1];
      rY[i] -= m * rY[i - 1];
    }

    points1[degree - 1] = new Pose(rX[degree - 1] / b[degree - 1],
        rY[degree - 1] / b[degree - 1]);
    for (int i = degree - 2; i >= 0; --i) {
      points1[i] = new Pose((rX[i] - (c[i] * points1[i + 1].getX())) / b[i],
          (rY[i] - (c[i] * points1[i + 1].getY())) / b[i]);
    }

    /* we have p1, now compute p2 */
    for (int i = 0; i < (degree - 1); i++) {
      points2[i] = new Pose((2 * knots.get(i + 1).getX()) - points1[i + 1].getX(),
          (2 * knots.get(i + 1).getY()) - points1[i + 1].getY());
    }

    points2[degree - 1] = new Pose(
        0.5 * (knots.get(degree).getX() + points1[degree - 1].getX()),
        0.5 * (knots.get(degree).getY() + points1[degree - 1].getY()));

    List<List<Pose>> controlPoints = new ArrayList<>(degree);

    for (int i = 0; i < degree; i++) {
      List<Pose> segmentControlPoints = new ArrayList<>(getPathNumberOfSteps());
      points1[0] = points1[0].rotate(knots.get(0), startAngle, isBackwards(), startScale);
      points2[degree - 1] = points2[degree - 1]
          .rotate(knots.get(knots.size() - 1), endAngle, !isBackwards(), endScale);
      Collections.addAll(segmentControlPoints, knots.get(i), points1[i], points2[i],
          knots.get(i + 1));
      Collections.addAll(controlPoints, segmentControlPoints);
    }
    return controlPoints;
  }

  /**
   * Stitches the bezier curve path data to make the single spline
   *
   * @param startPathData - requires the initial pathData
   */
  private void stitchPathData(PathData startPathData, List<List<Pose>> pathControlPoints) {
    double nextStartVelocity;
    double nextEndVelocity;
    PathData nextStartPathData = startPathData;
    getPathData().add(startPathData);
    ListIterator<List<Pose>> iterator = pathControlPoints.listIterator();
    while (iterator.hasNext()) {
      BezierCurve curve;
      nextStartVelocity = (iterator.nextIndex() == 0) ? startVelocity : getVCruise();
      nextEndVelocity =
          (iterator.nextIndex() == (pathControlPoints.size() - 1)) ? endVelocity
              : getVCruise();
      curve = new BezierCurve(getVCruise(), getAMax(), nextStartVelocity, nextEndVelocity,
          isBackwards(),
          nextStartPathData,
          iterator.next());
      getPathData().addAll(curve.getPathData());
      nextStartPathData = getPathData().get(getPathData().size() - 1);
    }
  }

  private List<DynamicBezierCurve> createBezierCurves(PathData startPathData,
      List<List<Pose>> pathControlPoints) {
    double nextStartVelocity;
    double nextEndVelocity;
    PathData nextStartPathData = startPathData;
    ListIterator<List<Pose>> iterator = pathControlPoints.listIterator();

    List<DynamicBezierCurve> bezierCurves = new ArrayList<>();

    while (iterator.hasNext()) {
      nextStartVelocity = (iterator.nextIndex() == 0) ? startVelocity : getVCruise();
      nextEndVelocity =
          (iterator.nextIndex() == (pathControlPoints.size() - 1)) ? endVelocity
              : getVCruise();

      DynamicBezierCurve curve = new DynamicBezierCurve(getVCruise(), getAMax(), nextStartVelocity,
          nextEndVelocity,
          isBackwards(),
          nextStartPathData,
          iterator.next());

      bezierCurves.add(curve);
      nextStartPathData = curve.getPathData(1.0).getLast();
    }

    return bezierCurves;
  }

  public void createPath() {
    List<List<Pose>> pathControlPoints = computeControlPoints(getKeyPoints());

    System.out.println(pathControlPoints.size());

    PathData startPathData = new PathData(new State(0, startVelocity, 0),
        new State(0, startVelocity, 0),
        new Pose(pathControlPoints.get(0).get(0).getX(), pathControlPoints.get(0).get(0).getY(),
            startAngle),
        0);

    bezierCurves = createBezierCurves(startPathData, pathControlPoints);
    System.out.println(bezierCurves.size());

    stitchPathData(startPathData, pathControlPoints);
  }

  public double getCurveLength() {
    return curveLength;
  }

  @Override
  public PathData createPathData(PathData previousPathData, double percentage) {
    return createPathData(percentage);
  }

  @Override
  public String toString() {
    return "Spline{" +
        "startAngle=" + startAngle +
        ", endAngle=" + endAngle +
        ", startScale=" + startScale +
        ", endScale=" + endScale +
        ", startVelocity=" + startVelocity +
        ", endVelocity=" + endVelocity +
        "} " + super.toString();
  }
}
