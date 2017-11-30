package org.waltonrobotics.motion;

import org.waltonrobotics.controller.Point;
import org.waltonrobotics.controller.Vector2;

public interface Path {

  public Vector2[] getSpeedVectors();

  public Point[] getPathPoints();

  public Point[] getLeftPath();

  public Point[] getRightPath();

  public double[] getDTsOnPath();
}
