package org.waltonrobotics.motion;

import static org.waltonrobotics.util.Helper.calculateCoefficients;
import static org.waltonrobotics.util.Helper.resizeArrayLeft;
import static org.waltonrobotics.util.Polynomial.calculateDerivative;
import static org.waltonrobotics.util.Polynomial.deconstructCoefficientsMatrix;
import static org.waltonrobotics.util.Polynomial.expandBinomial;
import static org.waltonrobotics.util.Polynomial.getPoint;
import static org.waltonrobotics.util.Polynomial.minimizeDistance;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.Pose;
import org.waltonrobotics.metadata.State;
import org.waltonrobotics.util.GaussLegendre;

/**
 * <p>This Path is a simple curve. The shape of the curve is controlled by the control points.
 * There are tangents from the first and second control points and the second to last and last
 * control points. Use this to make a straight line (use two control points) <br> <a
 * href=https://en.wikipedia.org/wiki/B%C3%A9zier_curve>Wikipedia page on Bezier Curves</a> <br> <a
 * href=https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html>Bezier curve
 * equations for N number of control points</a>
 * <p/>
 *
 * @author Marius Juston, Walton Robotics
 * @author Russell Newton, Walton Robotics
 */
public class BezierCurve extends Path {

  public static HashMap<Key, GaussLegendre> gaussLegendreHashMap = new HashMap<>();


  private final double startVelocity;
  private final double endVelocity;
  private final PathData startPathData;
  private final double startLCenter;
  public double curveLength;
  private List<Pose> pathPoints;
  private List<double[]> coefficients;

  /**
   * This constructor is used with the splines, but feel free to use it when creating your own
   * motions
   *
   * @param vCruise - the cruise velocity of the robot
   * @param aMax - the maximum acceleration of the robot
   * @param startVelocity - the start velocity
   * @param endVelocity - the end velocity
   * @param isBackwards - whether or not to move the robot backwards
   * @param startPathData - the starting PathData for the curve
   * @param controlPoints - the control points that define the curve
   */
  public BezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards,
      PathData startPathData, List<Pose> controlPoints) {
    super(vCruise, aMax, isBackwards, controlPoints);
    this.startVelocity = startVelocity;
    this.endVelocity = endVelocity;
    this.startPathData = startPathData;
    // The starting average encoder distance should always be 0
    startLCenter = startPathData.getLCenter();
    defineCoefficients();

    createPath();
  }

  /**
   * Use this if you don't need to define a starting PathData
   */
  public BezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards,
      List<Pose> controlPoints) {
    this(vCruise, aMax, startVelocity, endVelocity, isBackwards,
        (controlPoints.isEmpty()) ?
            new PathData(new Pose(0, 0), isBackwards) :
            ((controlPoints.size() == 1) ?
                new PathData(
                    new Pose(
                        controlPoints.get(0).getX(),
                        controlPoints.get(0).getY(),
                        controlPoints.get(0).getAngle())
                    , isBackwards) :
                new PathData(
                    new Pose(
                        controlPoints.get(0).getX(),
                        controlPoints.get(0).getY(),
                        StrictMath.atan2(
                            controlPoints.get(1).getY() - controlPoints.get(0).getY(),
                            controlPoints.get(1).getX() - controlPoints.get(0).getX())
                    )
                    , isBackwards)),
        controlPoints);
  }

  public BezierCurve(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards,
      Pose... controlPoints) {
    this(vCruise, aMax, startVelocity, endVelocity, isBackwards, Arrays.asList(controlPoints));
  }

//  public static void main(String[] args) {
//    BezierCurve curve = new BezierCurve(1, 1, 0, 0, false,
//        new Pose(0, 0), new Pose(1, -1), new Pose(2, -1), new Pose(3, 0));
//    System.out.println(curve.getClosestPose(new Pose(3, 0)).toString());
//  }

  private void defineCoefficients() {
    coefficients = new LinkedList<>();
    DMatrixRMaj coefficientsX = new DMatrixRMaj(keyPoints.size(), 1);
    DMatrixRMaj coefficientsY = new DMatrixRMaj(keyPoints.size(), 1);
    int[] binomialCoefficients = calculateCoefficients(keyPoints.size() - 1);
    for (int i = 0; i < keyPoints.size(); i++) {
      DMatrixRMaj expandedBinomial = new DMatrixRMaj(resizeArrayLeft(expandBinomial(1, -1, i),
          keyPoints.size()));
      DMatrixRMaj coefficientsIX = new DMatrixRMaj(keyPoints.size(), 1);
      DMatrixRMaj coefficientsIY = new DMatrixRMaj(keyPoints.size(), 1);
      for (int j = 0; j < keyPoints.size(); j++) {
        coefficientsIX.set(j,
            expandedBinomial.get(j) * keyPoints.get(keyPoints.size() - i - 1).getX()
                * binomialCoefficients[i]);
        coefficientsIY.set(j,
            expandedBinomial.get(j) * keyPoints.get(keyPoints.size() - i - 1).getY()
                * binomialCoefficients[i]);
      }
//      System.out.println(coefficientsIX.toString());
//      System.out.println(coefficientsIY.toString());
      CommonOps_DDRM.add(coefficientsX, coefficientsIX, coefficientsX);
      CommonOps_DDRM.add(coefficientsY, coefficientsIY, coefficientsY);
    }
    coefficients.add(deconstructCoefficientsMatrix(coefficientsX));
    coefficients.add(deconstructCoefficientsMatrix(coefficientsY));
  }

  public List<Pose> createPoints() {
    List<Pose> pathPoints = new LinkedList<>();

    for (double i = 0; i <= getPathNumberOfSteps(); i++) {

      pathPoints.add(getPoint(coefficients.get(0), coefficients.get(1),
          i / getPathNumberOfSteps()));
    }

    return pathPoints;
  }

  /**
   * Caluclates the length of the curve
   */
  public double getCurveLength() {
    double curveLength = 0;

    if (getKeyPoints().size() > 1) {
      for (int i = 1; i < getPathNumberOfSteps(); i++) {
        curveLength += pathPoints.get(i).distance(pathPoints.get(i - 1));
      }
    }

    return curveLength;
  }

  /**
   * @return the degree of the curve
   */
  public int getDegree() {
    return getKeyPoints().size() - 1;
  }


  /**
   * Creates the PathData list
   */
  private void setData(PathData startData) {

    for (int i = 1; i <= getPathNumberOfSteps(); i++) {
      startData = calculateData(startData, pathPoints.get(i));
      getPathData().add(startData);
    }

  }

  /**
   * @param nextPosition - The Pose of the PathData to calculate on
   * @return a new PathData from the calculations
   */
  private PathData calculateData(PathData currentPathData, Pose nextPosition) {
    Pose previousCenter = currentPathData.getCenterPose();
    State previousLeft = currentPathData.getLeftState();
    State previousRight = currentPathData.getRightState();
    double previousTime = currentPathData.getTime();
    double previousLCenter = currentPathData.getLCenter();
    // When cruising, acceleration is 0
    double acceleration = 0;

    // The change in angle of the robot
    double dAngle = Path.boundAngle(nextPosition.getAngle() - previousCenter.getAngle());

    // The change in distance of the robot sides
    // FIXME This is probably wrong dLength should be 0 if there is not angle
//		double dLength = (previousCenter.sameCoordinates(nextPosition)? 0: previousCenter.distance(nextPosition)) * (isBackwards() ? -1 : 1);
    double dLength = previousCenter.distance(nextPosition) * (isBackwards() ? -1 : 1);
    double dlLeft = dLength - ((dAngle * getRobotWidth()) / 2.0);
    double dlRight = dLength + ((dAngle * getRobotWidth()) / 2.0);

    // The time required to get to the next point
    double dTime = Math.max(Math.abs(dlLeft), Math.abs(dlRight)) / getVCruise();
    // The hypothetical velocity to get to that point
    double velocity = dLength / dTime;

    // The average encoder distance to the next point
    double lCenter = (previousLCenter + (0.5 * dLength)) - startLCenter;
    double vAccelerating;
    double vDecelerating;

    if (isBackwards()) {
      vAccelerating = -Math
          .sqrt(StrictMath.pow(startVelocity, 2.0) + (getAMax() * Math.abs(lCenter)));
      vDecelerating = -Math
          .sqrt(
              StrictMath.pow(endVelocity, 2.0) + (getAMax() * Math
                  .abs(curveLength - Math.abs(lCenter))));
      if ((vAccelerating > velocity) && (vAccelerating > vDecelerating)) {
        acceleration = -getAMax();
        dTime = dLength / vAccelerating;
      }
      if ((vDecelerating > velocity) && (vDecelerating > vAccelerating)) {
        acceleration = getAMax();
        dTime = dLength / vDecelerating;
      }
    } else {
      vAccelerating = Math
          .sqrt(StrictMath.pow(startVelocity, 2.0) + (getAMax() * Math.abs(lCenter)));
      vDecelerating = Math
          .sqrt(
              StrictMath.pow(endVelocity, 2.0) + (getAMax() * Math
                  .abs(curveLength - Math.abs(lCenter))));

      if ((vAccelerating < velocity) && (vAccelerating < vDecelerating)) {
        acceleration = getAMax();
        dTime = dLength / vAccelerating;
      }
      if ((vDecelerating < velocity) && (vDecelerating < vAccelerating)) {
        acceleration = -getAMax();
        dTime = dLength / vDecelerating;
      }
    }
//		System.out
//			.println("acc: " + vAccelerating + " dec: " + vDecelerating + " vel: " + velocity + " velAct: " + dLength/dTime);

    double velocityL = dlLeft / dTime;
    double velocityR = dlRight / dTime;

    State left = new State(previousLeft.getLength() + dlLeft, velocityL, acceleration);
    State right = new State(previousRight.getLength() + dlRight, velocityR, acceleration);
    Pose center = new Pose(nextPosition.getX(), nextPosition.getY(),
        previousCenter.getAngle() + dAngle);
    return new PathData(left, right, center, previousTime + dTime);
  }

  public void createPath() {
    pathPoints = createPoints();
    curveLength = getCurveLength();
    setData(startPathData);
  }

  public double computeArcLength() {
    return computeArcLength(16 /* this number seems to give decent accuracy*/, 0, 1.0);
  }

  public double computeArcLength(int n) {

    return computeArcLength(n, 0, 1.0);
  }

  public double computeArcLength(double lowerBound, double upperBound) {
    return computeArcLength(16, lowerBound, upperBound);
  }

  /**
   * Uses the Gauss Legendre integration to approximate the arc length of the Bezier curve. This is
   * the fastest technique (faster than sampling) when having a large path and shows the most
   * accurate results
   *
   * @param n the number of integral strips 2+ more means better accuracy
   * @param lowerBound the lower bound to integrate (inclusive) [0,1]
   * @param upperBound the upper bound to integrate (inclusive) [0,1]
   * @return the arc length of the Bezier curve of t range of [lowerBound, upperBound]
   */
  public double computeArcLength(int n, double lowerBound, double upperBound) {

    GaussLegendre gl;

    Key key = new Key(n, upperBound, lowerBound);
    if (gaussLegendreHashMap.containsKey(key)) {
      gl = gaussLegendreHashMap.get(key);
    } else {
      gl = new GaussLegendre(n, lowerBound, upperBound);
      gaussLegendreHashMap.put(key, gl);
    }

    double[] t = gl.getNodes();
    double[] C = gl.getWeights();

    double sum = 0;

    for (int i = 0; i < t.length; i++) {

      double dx = calculateDerivative(coefficients.get(0), t[i]);
      double dy = calculateDerivative(coefficients.get(1), t[i]);
      sum += C[i] * StrictMath.hypot(dx, dy);
    }

    return sum;
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

    addKeyPoints(stringBuilder);
    return stringBuilder.toString();
  }


  @Override
  public String toString() {
    return "BezierCurve{" +
        "startVelocity=" + startVelocity +
        ", endVelocity=" + endVelocity +
        ", startPathData=" + startPathData +
        ", startLCenter=" + startLCenter +
        ", pathPoints=" + pathPoints +
        ", curveLength=" + curveLength +
        "} " + super.toString();
  }

  /**
   * Finds the closest Pose in the curve to {@code inputPose}. This procedure is an optimization
   * problem to find t where the distance between B(t) and {@code inputPose} is minimized. This
   * works only for cubic curves.
   */
  public Pose getClosestPose(Pose inputPose) {
    return minimizeDistance(inputPose, 0, 1, coefficients.get(0),
        coefficients.get(1));
  }

  public double getStartVelocity() {
    return startVelocity;
  }

  public double getEndVelocity() {
    return endVelocity;
  }

  public static class Key {

    int n;
    double upper;
    double lower;

    public Key(int n, double upper, double lower) {
      this.n = n;
      this.upper = upper;
      this.lower = lower;
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) {
        return true;
      }
      if ((o == null) || (getClass() != o.getClass())) {
        return false;
      }

      Key key = (Key) o;

      if (n != key.n) {
        return false;
      }
      if (Double.compare(key.upper, upper) != 0) {
        return false;
      }
      return Double.compare(key.lower, lower) == 0;
    }

    @Override
    public int hashCode() {
      int result;
      long temp;
      result = n;
      temp = Double.doubleToLongBits(upper);
      result = (31 * result) + (int) (temp ^ (temp >>> 32));
      temp = Double.doubleToLongBits(lower);
      result = (31 * result) + (int) (temp ^ (temp >>> 32));
      return result;
    }
  }
}
