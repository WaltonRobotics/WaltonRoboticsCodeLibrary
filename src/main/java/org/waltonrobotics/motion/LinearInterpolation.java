package org.waltonrobotics.motion;

import java.util.List;
import org.waltonrobotics.metadata.Pose;

/**
 * @author Russell Newton
 **/
public class LinearInterpolation extends Path {

  private final double vCruise;
  private final double aMax;
  private final double startVelocity;
  private final double endVelocity;
  private final boolean isBackwards;
  private final List<Pose> knots;

  /**
   * Create a {@code LinearInterpolation Path}.
   *
   * @param vCruise - max velocity
   * @param aMax - max acceleration
   * @param startVelocity - the starting velocity of the Path
   * @param endVelocity - the ending velocity of the Path
   * @param isBackwards - if the robot will be moving backwards, make this true
   * @param knots - the points you want the robot to drive through
   */
  public LinearInterpolation(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards, List<Pose> knots) {
    super(vCruise, aMax, false, knots);
    this.vCruise = vCruise;
    this.aMax = aMax;
    this.startVelocity = startVelocity;
    this.endVelocity = endVelocity;
    this.isBackwards = isBackwards;
    this.knots = knots;
    createPath();
  }

  private void createPath() {
    Line segment = new Line(vCruise, aMax, startVelocity, vCruise, isBackwards, knots.get(0),
        knots.get(1));
    for (int i = 1; i < knots.size() - 2; i++) {
      getPathData().addAll(segment.getPathData());
      segment = new Line(vCruise, aMax,
          segment.getPathData().getFirst().getLeftState().getVelocity(),
          vCruise, isBackwards, knots.get(i),
          knots.get(i + 1));
    }
    segment = new Line(vCruise, aMax, getPathData().getLast().getLeftState().getVelocity(),
        endVelocity, isBackwards, knots.get(knots.size() - 2), knots.get(knots.size() - 1));
    getPathData().addAll(segment.getPathData());
  }

}
