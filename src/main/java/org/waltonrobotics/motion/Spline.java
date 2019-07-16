package org.waltonrobotics.motion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.Map.Entry;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.State;

/**
 * This path is a spline that will go through the set knots by stitching together several Bezier
 * curves. By default, it will try to make the shortest path possible, but the start and end angles
 * (degrees) indicate how the robot is facing or how you want it to face. This is not very effective
 * with only 2 knots. If you want a straight line, make a Bezier Curve. <br> <a
 * href=https://www.particleincell.com/2012/bezier-splines>Interactive javascript spline</a> <br> <a
 * href=https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm>Thomas algorithm</a>
 *
 * @author Russell Newton, Walton Robotics
 */

public class Spline extends Path {

  private final double startAngle;
  private final double endAngle;
  private final double startScale;
  private final double endScale;
  private final double startVelocity;
  private final double endVelocity;
  private final double curveLength;
  private List<BezierCurve> definingBezierCurves = new ArrayList<>();

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
  public Spline(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards, double scaleStart, double scaleEnd,
      List<Pose> knots) {
    super(vCruise, aMax, isBackwards, knots);

    if (knots.isEmpty()) {
      startAngle = 0;
      endAngle = 0;
    } else {
      this.startAngle = knots.get(0).getAngle();
      this.endAngle = knots.get(knots.size() - 1).getAngle();
    }
    startScale = scaleStart;
    endScale = scaleEnd;
    this.endVelocity = endVelocity;
    this.startVelocity = startVelocity;

    createPath();
    curveLength = getDefiningBezierCurves().stream().mapToDouble(BezierCurve::getCurveLength).sum();
  }

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
  public Spline(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards, Pose... knots) {
    this(vCruise, aMax, startVelocity, endVelocity,
        isBackwards, 1.0, 1.0, Arrays.asList(knots));
  }

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
  public Spline(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards, List<Pose> knots) {
    this(vCruise, aMax, startVelocity, endVelocity,
        isBackwards, 1.0, 1.0, knots);
  }

  public double getCurveLength() {
    return curveLength;
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
    b[0] = 2.0;
    c[0] = 1.0;
    rX[0] = knots.get(0).getX() + (2.0 * knots.get(1).getX());
    rY[0] = knots.get(0).getY() + (2.0 * knots.get(1).getY());

    /* internal segments */
    for (int i = 1; i < (degree - 1); i++) {
      a[i] = 1.0;
      b[i] = 4.0;
      c[i] = 1.0;
      rX[i] = (4.0 * knots.get(i).getX()) + (2.0 * knots.get(i + 1).getX());
      rY[i] = (4.0 * knots.get(i).getY()) + (2.0 * knots.get(i + 1).getY());
    }

    /* right segment */
    a[degree - 1] = 2.0;
    b[degree - 1] = 7.0;
    c[degree - 1] = 0;
    rX[degree - 1] = (8.0 * knots.get(degree - 1).getX()) + knots.get(degree).getX();
    rY[degree - 1] = (8.0 * knots.get(degree - 1).getY()) + knots.get(degree).getY();

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
      points2[i] = new Pose((2.0 * knots.get(i + 1).getX()) - points1[i + 1].getX(),
          (2.0 * knots.get(i + 1).getY()) - points1[i + 1].getY());
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

  private List<BezierCurve> createBezierCurves(PathData startPathData,
      List<List<Pose>> pathControlPoints) {
    double nextStartVelocity;
    double nextEndVelocity;
    PathData nextStartPathData = startPathData;
    ListIterator<List<Pose>> iterator = pathControlPoints.listIterator();

    List<BezierCurve> bezierCurves = new ArrayList<>();

    while (iterator.hasNext()) {
      nextStartVelocity = (iterator.nextIndex() == 0) ? startVelocity : getVCruise();
      nextEndVelocity =
          (iterator.nextIndex() == (pathControlPoints.size() - 1)) ? endVelocity
              : getVCruise();

      BezierCurve curve = new BezierCurve(getVCruise(), getAMax(), nextStartVelocity,
          nextEndVelocity,
          isBackwards(),
          nextStartPathData,
          iterator.next());

//			System.out.println(curve.getKeyPoints().size());
      bezierCurves.add(curve);
      nextStartPathData = curve.getPathData().getLast();
    }

    return bezierCurves;
  }

  public void createPath() {
    List<List<Pose>> pathControlPoints = computeControlPoints(getKeyPoints());

//		System.out.println(pathControlPoints.size());

    PathData startPathData = new PathData(new State(0, startVelocity, 0),
        new State(0, startVelocity, 0),
        new Pose(pathControlPoints.get(0).get(0).getX(), pathControlPoints.get(0).get(0).getY(),
            startAngle),
        0);

    definingBezierCurves = createBezierCurves(startPathData, pathControlPoints);
//		System.out.println(bezierCurves.size());

    stitchPathData(startPathData, pathControlPoints);
  }

  public List<BezierCurve> getDefiningBezierCurves() {
    return definingBezierCurves;
  }

  @Override
  public String convertToString() {

    String className = getClass().getName();

    StringBuilder stringBuilder = new StringBuilder();
    stringBuilder.append(className);
    stringBuilder.append(' ');
    stringBuilder.append(getVCruise());
    stringBuilder.append(' ');
    stringBuilder.append(getAMax());
    stringBuilder.append(' ');
    stringBuilder.append(startVelocity);
    stringBuilder.append(' ');
    stringBuilder.append(endVelocity);
    stringBuilder.append(' ');
    stringBuilder.append(isBackwards());
    stringBuilder.append(' ');
    stringBuilder.append(endScale);
    stringBuilder.append(' ');
    stringBuilder.append(startScale);

    stringBuilder.append(' ');

    addKeyPoints(stringBuilder);
    return stringBuilder.toString();


  }

//  public Pose getPoint(double percentage) {
////    TODO make this work and get the correct point from the inner bezier curves given the percent t
//
////    System.out.println(percentage);
////    int numberOfCurves = getDefiningBezierCurves().size();
////
////    int currentlyOn = (int) (percentage * (numberOfCurves - 1));
//////    System.out.println(currentlyOn);
////
//////    double percentageInCurve = 1- (percentage * numberOfCurves);
////    double percentageInCurve = percentage * (numberOfCurves - currentlyOn);
//////    System.out.println(percentageInCurve);
////
////    double percent = 1.0 / numberOfCurves;
//////    System.out.println(percent);
////
////    double percentagePerBezierCurve = percentageInCurve / percent;
//////    System.out.println(percentagePerBezierCurve);
////
////    return getDefiningBezierCurves().get(currentlyOn).getPoint(percentagePerBezierCurve);
//
//    if (percentage != 1.0) {
//      int value = (int) (percentage * getDefiningBezierCurves().size());
//      BezierCurve bezierCurve = getDefiningBezierCurves()
//          .get(value);
//
//      return bezierCurve
//          .getPoint(((percentage * getDefiningBezierCurves().size()) - value));
//    }
//    return definingBezierCurves.get(definingBezierCurves.size() - 1).getPoint(1.0);
//  }

//  public Pose getDerivative(double percentage) {
//    if (percentage != 1.0) {
//      int value = (int) (percentage * getDefiningBezierCurves().size());
//      BezierCurve bezierCurve = getDefiningBezierCurves()
//          .get(value);
//
//      return bezierCurve
//          .getDerivative(((percentage * getDefiningBezierCurves().size()) - value));
//    }
//    return definingBezierCurves.get(definingBezierCurves.size() - 1).getDerivative(1.0);
//  }

  /**
   * Returns the point on the path that is closest to initPose.
   */
  public Pose getClosestPose(Pose inputPose) {
    HashMap<Pose, Double> distanceMap = new HashMap<>();
    for (BezierCurve curve : getDefiningBezierCurves()) {
      Pose closestCurvePoint = curve.getClosestPose(inputPose);
      distanceMap.put(closestCurvePoint, inputPose.distance(closestCurvePoint));
    }
    return distanceMap.entrySet().stream().sorted(Entry.comparingByValue()).
        map(Entry::getKey).toArray(Pose[]::new)[0];
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
