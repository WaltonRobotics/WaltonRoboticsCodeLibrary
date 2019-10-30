package org.waltonrobotics.util;

import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.sin;
import static org.waltonrobotics.util.Helper.calculateCoefficients;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map.Entry;
import org.ejml.data.Complex_F64;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.waltonrobotics.metadata.Pose;

/**
 * Contains functions to manage 2D Polynomials.
 *
 * @author Russell Newton, Walton Robotics
 **/
public class Polynomial {

  private final double[] coefficientsX;
  private final double[] coefficientsY;

  /**
   * Create a new parametric {@code Polynomial} with the passed coefficients. Coefficients should be
   * passed in from least to most significant.
   */
  public Polynomial(double[] coefficientsX, double[] coefficientsY) {
    this.coefficientsX = coefficientsX;
    this.coefficientsY = coefficientsY;
  }

  /**
   * Create a new {@code Polynomial} as a function of x.
   */
  public Polynomial(double[] coefficients) {
    this(new double[]{0.0, 1.0}, coefficients);
  }

  /**
   * Multiplies two polynomials, returning the resulting coefficients.
   */
  public static DMatrixRMaj multiply(double[] coefficientsA, double[] coefficientsB) {
    //Expand polynomial multiplication
    DMatrixRMaj vertical = new DMatrixRMaj(coefficientsA.length, 1, true, coefficientsA);
    DMatrixRMaj horizontal = new DMatrixRMaj(1, coefficientsB.length, true, coefficientsB);
    DMatrixRMaj product = new DMatrixRMaj(coefficientsA.length, coefficientsB.length);
    CommonOps_DDRM.mult(vertical, horizontal, product);

    //Combine like terms
    double[] newCoefficients = new double[coefficientsA.length + coefficientsB.length - 1];
    for (int k = 0; k < newCoefficients.length; k++) {
      for (int j = 0; j <= k; j++) {
        int i = k - j;
        if (i < coefficientsA.length && j < coefficientsB.length) {
          newCoefficients[k] += product.unsafe_get(i, j);
        }
      }
    }
//    System.out.println(Arrays.toString(newCoefficients));
    return new DMatrixRMaj(newCoefficients);
  }

  /**
   * Compute the coefficients of the polynomial resulting from squaring the input polynomial.
   *
   * @param coefficients Coefficients of the input polynomial, from least to most significant.
   * @return The resulting coefficients, from least to most significant.
   */
  public static DMatrixRMaj square(double... coefficients) {
    return multiply(coefficients, coefficients);
  }

  /**
   * Find the coefficients of the derivative of a polynomial.
   *
   * @param coefficients The input coefficients.
   * @return The coefficients of the derivative polynomial.
   */
  public static DMatrixRMaj derivativeCoefficients(DMatrixRMaj coefficients) {
    double[] newCoefficients = new double[coefficients.numRows - 1];
    for (int i = 1; i < coefficients.numRows; i++) {
      newCoefficients[i - 1] = coefficients.get(i) * i;
    }
    return new DMatrixRMaj(newCoefficients);
  }

  /**
   * <p>
   * Given a set of polynomial coefficients, compute the roots of the polynomial.  Depending on the
   * polynomial being considered the roots may contain complex number.  When complex numbers are
   * present they will come in pairs of complex conjugates.
   * </p>
   *
   * <p>
   * Coefficients are ordered from least to most significant, e.g: y = c[0] + x*c[1] + x*x*c[2].
   * </p><br>
   *
   * <a href="https://ejml.org/wiki/index.php?title=Example_Polynomial_Roots">Source</a>
   *
   * @param coefficients Coefficients of the polynomial.
   * @return The roots of the polynomial
   */
  public static Complex_F64[] findRoots(double... coefficients) {
//    System.out.println(Arrays.toString(coefficients));
    coefficients = removeTrailingZeros(coefficients);
    int N = coefficients.length - 1;

    // Construct the companion matrix
    DMatrixRMaj c = new DMatrixRMaj(N, N);

    double a = coefficients[N];
    for (int i = 0; i < N; i++) {
      c.set(i, N - 1, -coefficients[i] / a);
    }
    for (int i = 1; i < N; i++) {
      c.set(i, i - 1, 1);
    }

    // use generalized eigenvalue decomposition to find the roots
    EigenDecomposition_F64<DMatrixRMaj> evd = DecompositionFactory_DDRM.eig(N, false);

    evd.decompose(c);

    Complex_F64[] roots = new Complex_F64[N];

    for (int i = 0; i < N; i++) {
      roots[i] = evd.getEigenvalue(i);
//      System.out.println(roots[i].toString());
    }

    return roots;
  }

  /**
   * Removes trailing zeros in a coefficients array. Necessary to ensure success with {@code
   * findRoots}.
   */
  private static double[] removeTrailingZeros(double[] coefficients) {
    int trailingZeros = 0;
    for (int i = coefficients.length - 1; i >= 0; i--) {
      if (coefficients[i] != 0) {
        break;
      }
      trailingZeros++;
    }
    double[] newCoefficients = new double[coefficients.length - trailingZeros];
    for (int i = 0; i < newCoefficients.length; i++) {
      newCoefficients[i] = coefficients[i];
    }
    return newCoefficients;
  }

  /**
   * Deconstructs a single row {@code DMatrixRMaj} into a double[]
   */
  public static double[] deconstructCoefficientsMatrix(DMatrixRMaj coefficients) {
    double[] newCoefficients = new double[coefficients.numCols];
    for (int i = 0; i < coefficients.numCols; i++) {
      newCoefficients[i] = coefficients.get(i);
    }
    return newCoefficients;
  }

  public static double[][] deconstructMatrix(DMatrixRMaj matrix) {
    double[][] newMatrix = new double[matrix.getNumRows()][matrix.getNumCols()];
    for (int i = 0; i < matrix.getNumRows(); i++) {
      for (int j = 0; j < matrix.getNumCols(); j++) {
        newMatrix[i][j] = matrix.get(i, j);
      }
    }
    return newMatrix;
  }

  /**
   * Finds t corresponding to the point closest to {@code inputPoint} on the parametric {@code
   * Polynomial} curve defined by {@code coefficientsX} and {@code coefficientsY}.
   *
   * @param lowBound low bound for t
   * @param highBound high bound for t
   */
  public static Pose minimizeDistance(Pose inputPose, double lowBound, double highBound,
      double[] coefficientsX, double[] coefficientsY) {
    //Define coefficients for distance minimization
    double[] tempX = coefficientsX.clone();
    double[] tempY = coefficientsY.clone();
    tempX[0] -= inputPose.getX();
    tempY[0] -= inputPose.getY();
    DMatrixRMaj minimizationCoefficientsX = derivativeCoefficients(square(tempX));
    DMatrixRMaj minimizationCoefficientsY = derivativeCoefficients(square(tempY));
    int combinedCoefficientsLength = Math.max(minimizationCoefficientsX.numRows,
        minimizationCoefficientsY.numRows);
    DMatrixRMaj minimizationCoefficients = new DMatrixRMaj(combinedCoefficientsLength, 1);
    minimizationCoefficientsX.reshape(combinedCoefficientsLength, 1, true);
    minimizationCoefficientsY.reshape(combinedCoefficientsLength, 1, true);
    CommonOps_DDRM.add(minimizationCoefficientsX, minimizationCoefficientsY,
        minimizationCoefficients);
    double[] coefficients = deconstructCoefficientsMatrix(minimizationCoefficients);

    //Find t such that the dot-product is minimized. We can safely assume that there will be at
    // least 1 real root.
    Complex_F64[] roots = findRoots(coefficients);
//    System.out.println(Arrays.toString(roots));
    double[] realRoots = Arrays.stream(roots).filter(Complex_F64::isReal)
        .mapToDouble(Complex_F64::getReal).toArray();

    //<Root, Distance to Curve>
    HashMap<Double, Double> distancesMap = new HashMap<>();
    distancesMap
        .put(lowBound, inputPose.distance(getPoint(coefficientsX, coefficientsY, lowBound)));
    distancesMap.put(highBound, inputPose.distance(getPoint(coefficientsX, coefficientsY,
        highBound)));
    for (double root : realRoots) {
      distancesMap.put(root, inputPose.distance(getPoint(coefficientsX, coefficientsY, root)));
    }
    realRoots = distancesMap.entrySet().stream().filter(i -> i.getKey() <= highBound &&
        i.getKey() >= lowBound).sorted(Entry.comparingByValue()).mapToDouble(Entry::getKey).
        toArray();

    return getPoint(coefficientsX, coefficientsY, realRoots[0]);
  }

  /**
   * Finds x corresponding to the point closest to {@code inputPoint} on the parametric {@code
   * Polynomial} curve defined by {@code coefficients}.
   *
   * @param lowBound low bound for x
   * @param highBound high bound for x
   */
  public static Pose minimizeDistance(Pose inputPose, double lowBound, double highBound,
      double[] coefficients) {
    return minimizeDistance(inputPose, lowBound, highBound, new double[]{0, 1},
        coefficients);
  }

  /**
   * Returns the point on a parametric at {@code t}.
   *
   * @param coefficientsX the coefficients defining the x polynomial, from least significant to
   * most.
   * @param coefficientsY the coefficients defining the y polynomial, from least significant to
   * most.
   */
  public static Pose getPoint(double[] coefficientsX, double[] coefficientsY, double t) {
    double x = calculateFunction(coefficientsX, t);
    double y = calculateFunction(coefficientsY, t);
    double dX = calculateDFDT(coefficientsX, t);
    double dY = calculateDFDT(coefficientsY, t);

    return new Pose(x, y, Math.atan2(dY, dX));
  }

  /**
   * Returns the point on a function at {@code t}.
   *
   * @param coefficients the coefficients defining the function
   */
  public static Pose getPoint(double[] coefficients, double t) {
    return getPoint(new double[]{0, 1}, coefficients, t);
  }

  /**
   * Calculates f(x).
   *
   * @param coefficients coefficients defining f(x), from least to most significant
   */
  private static double calculateFunction(double[] coefficients, double x) {
    double y = 0;
    for (int i = 0; i < coefficients.length; i++) {
      y += coefficients[i] * Math.pow(x, i);
    }
    return y;
  }

  /**
   * Calculates dy/dx(t). Coefficients are ordered from least to most significant.
   */
  public static double calculateDYDX(double[] coefficientsX, double[] coefficientsY, double t) {
    double dx = calculateDFDT(coefficientsX, t);
    double dy = calculateDFDT(coefficientsY, t);
    return dy / dx;
  }

  /**
   * Calculates f'(t).
   *
   * @param coefficients coefficients defining f'(t), from least to most significant
   */
  public static double calculateDFDT(double[] coefficients, double t) {
    return calculateFunction(deconstructCoefficientsMatrix(derivativeCoefficients(
        new DMatrixRMaj(coefficients))), t);
  }

  /**
   * Expand binomial power of form (a + bx)^n, returning resulting coefficients.
   */
  public static double[] expandBinomial(double a, double b, int n) {
    int[] binomialCoefficients = calculateCoefficients(n);
    double[] aPowers = new double[binomialCoefficients.length];
    double[] bPowers = new double[binomialCoefficients.length];
    double[] expandedCoefficients = new double[binomialCoefficients.length];
    for (int i = 0; i < binomialCoefficients.length; i++) {
      aPowers[i] = Math.pow(a, binomialCoefficients.length - i);
      bPowers[i] = Math.pow(b, i);
    }
    for (int i = 0; i < binomialCoefficients.length; i++) {
      expandedCoefficients[i] = binomialCoefficients[i] * aPowers[i] * bPowers[i];
    }
    return expandedCoefficients;
  }

  /**
   * Reflect a {@code Polynomial}, defined by {@code coefficientsX} and {@code coefficientsY}
   * (sorted by increasing significance), over a line through {@code through}, with a slope defined
   * by {@code through.getAngle()}.
   */
  public static Polynomial reflect(double[] coefficientsX, double[] coefficientsY, Pose through) {
    DMatrixRMaj function = new DMatrixRMaj(new double[][]{coefficientsX, coefficientsY});
    double yIntercept = -through.getX() * Math.tan(through.getAngle()) + through.getY();
    DMatrixRMaj shiftDown = new DMatrixRMaj(new double[][]{
        {1, 0, 0},
        {0, 1, -yIntercept},
        {0, 0, 1}
    });
    DMatrixRMaj reflect = new DMatrixRMaj(new double[][]{
        {cos(2 * through.getAngle()), sin(2 * through.getAngle()), 0},
        {sin(2 * through.getAngle()), -cos(2 * through.getAngle()), 0},
        {1, 0, 0}
    });
    DMatrixRMaj shiftUp = new DMatrixRMaj(new double[][]{
        {1, 0, 0},
        {0, 1, -yIntercept},
        {0, 0, 1}
    });
    DMatrixRMaj reflectedFunction = shiftUp.copy();
    CommonOps_DDRM.mult(reflectedFunction, reflect, reflectedFunction);
    CommonOps_DDRM.mult(reflectedFunction, shiftDown, reflectedFunction);
    CommonOps_DDRM.mult(reflectedFunction, function, reflectedFunction);
    double[][] reflectedCoefficients = deconstructMatrix(reflectedFunction);
    return new Polynomial(reflectedCoefficients[0], reflectedCoefficients[1]);
  }

  /**
   * Reflect this {@code Polynomial} over a line through {@code through}, with a slope defined by
   * {@code through.getAngle()}.
   */
  public Polynomial reflect(Pose through) {
    return reflect(coefficientsX, coefficientsY, through);
  }

  /**
   * Finds t corresponding to the point closest to {@code inputPoint} on this {@code Polynomial}.
   *
   * @return the {@code Pose} at the calculated t.
   */
  public Pose minimizeDistance(Pose inputPose, double lowBound, double highBound) {
    return minimizeDistance(inputPose, lowBound, highBound, coefficientsX, coefficientsY);
  }

  /**
   * @return f(t).
   */
  public Pose getPoint(double t) {
    return getPoint(coefficientsX, coefficientsY, t);
  }

  /**
   * Calculates dy/dt(t) of this {@code Polynomial}.
   */
  public double calculateDYDT(double t) {
    return calculateDFDT(coefficientsY, t);
  }

  /**
   * @return dx/dt(t).
   */
  public double calculateDXDT(double t) {
    return calculateDFDT(coefficientsX, t);
  }

  /**
   * @return dy/dx(t).
   */
  public double calculateDYDX(double t) {
    return calculateDYDX(coefficientsX, coefficientsY, t);
  }

  /**
   * @return this * {@code factor}.
   */
  public Polynomial multiply(Polynomial factor) {
    DMatrixRMaj prodX = Polynomial.multiply(this.coefficientsX, factor.coefficientsX);
    DMatrixRMaj prodY = Polynomial.multiply(this.coefficientsX, factor.coefficientsX);
    return new Polynomial(deconstructCoefficientsMatrix(prodX),
        deconstructCoefficientsMatrix(prodY));
  }

}
