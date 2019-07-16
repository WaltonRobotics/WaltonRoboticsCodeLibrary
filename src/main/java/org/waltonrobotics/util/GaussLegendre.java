package org.waltonrobotics.util;

import java.util.Arrays;

public class GaussLegendre {

  private double[] weights;
  private double[] nodes;
  private double lowerBound;
  private double upperBound;

  /**
   * Constructs nodes and weights for an n-dimensional Gauss Legendre quadrature between on the
   * interval [lowerBound, upperBound]. The constructor creates nodes and weights and then
   * initializes all fields of the instance.
   *
   * @param numberOfPoints, lowerBound, upperBound
   * @author Alessandro Gnoatto based on Matlab code by Greg von Winckel.
   */
  public GaussLegendre(int numberOfPoints, double lowerBound, double upperBound) {

    numberOfPoints -= 1;
    int n1 = numberOfPoints + 1;
    int n2 = numberOfPoints + 2;

    double h = 2.0 / (n1 - 1);

    double[] xu = new double[n1];
    for (int i = 0; i < n1; i++) {
      xu[i] = -1.0 + (h * i);
    }

    //Initial guess
    double[] y = new double[numberOfPoints + 1];
    double[] y0 = new double[numberOfPoints + 1];
    for (int i = 0; i <= numberOfPoints; i++) {
      y[i] =
          StrictMath.cos((((2.0 * i) + 1.0) * Math.PI) / ((2 * numberOfPoints) + 2)) + ((0.27 / n1)
              * StrictMath
              .sin((Math.PI * xu[i] * numberOfPoints) / n2));
      y0[i] = 2.0;

    }

    /*
     * Compute the zeros of the N+1 Legendre Polynomial
     * using the recursion relation and the Newton-Raphson method
     */

    double eps = 2.22E-16;
    //Legendre-Gauss Vandermonde Matrix
    double[][] L = new double[n1][n2];
    double[] Lp = new double[n1];

    //Iterate until new points are uniformly within epsilon of old points
    while (maxDistance(y, y0) > eps) {

      for (int j = 0; j < n1; j++) {
        L[j][0] = 1.0;
        L[j][1] = y[j];
        y0[j] = y[j];
      }

      for (int k = 1; k < n1; k++) {
        for (int j = 0; j < n1; j++) {
          L[j][k + 1] =
              ((((2.0 * (k + 1)) - 1.0) * y[j] * L[j][k]) - ((k) * L[j][k - 1])) / (k + 1);
        }
      }

      for (int j = 0; j < n1; j++) {
        //Lp(j)=(N2)*( L(j,N1)-y(j).*L(j,N2) )./(1-y(j).^2);
        Lp[j] = ((n2) * (L[j][n1 - 1] - (y[j] * L[j][n2 - 1]))) / (1.0 - (y[j] * y[j]));
        //y(j)=y(j)-L(j,N2)./Lp(j);

        y[j] -= (L[j][n2 - 1] / Lp[j]);
      }

    }

    double[] x = new double[n1];
    double[] w = new double[n1];
    double[] xInverted = new double[n1];

    for (int j = 0; j < n1; j++) {
      //Linear map from[-1,1] to [a,b]
      //x=(a*(1-y)+b*(1+y))/2;
      x[j] = ((lowerBound * (1.0 - y[j])) + (upperBound * (1.0 + y[j]))) / 2.0;
      xInverted[n1 - 1 - j] = x[j]; //TODO Change this to make the nodes reversed
      //corresponding weights
      //w=(b-a)./((1-y.^2).*Lp.^2)*(N2/N1)^2;
      w[j] = ((upperBound - lowerBound) / ((1.0 - (y[j] * y[j])) * Lp[j] * Lp[j])) * StrictMath
          .pow(((double) n2) / n1, 2.0);
    }

    this.lowerBound = lowerBound;
    this.upperBound = upperBound;
    this.weights = w;
    this.nodes = xInverted;
  }

  public static void main(String[] args) {

    int n = 5;
    GaussLegendre gl = new GaussLegendre(n, -1.0, 1.0);

    System.out.println("Gauss Legendre nodes for n =" + n + " between 0 and 1");
    for (int j = 0; j < n; j++) {
      System.out.println(gl.getNodes()[j]);
    }
    System.out.println("Gauss Legendre weights for n =" + n + " between 1 and 1");
    for (int j = 0; j < n; j++) {
      System.out.println(gl.getWeights()[j]);
    }

  }

  public double[] getWeights() {
    return this.weights;
  }

  public double[] getNodes() {
    return this.nodes;
  }

  public double getLowerBound() {
    return this.lowerBound;
  }

  public double getUpperBound() {
    return this.upperBound;
  }

  /**
   * @return max(abs ( a[] - b[]))
   */
  private double maxDistance(double[] a, double[] b) {

    int n = a.length;
    double[] distance = new double[n];

    for (int i = 0; i < n; i++) {
      distance[i] = Math.abs(a[i] - b[i]);
    }
    //sort element of the array
    Arrays.sort(distance);
    //return the largest element which is stored at the end
    return distance[distance.length - 1];
  }


}

