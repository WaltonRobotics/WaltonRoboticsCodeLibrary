package org.waltonrobotics.metadata;

/**
 * @author Russell Newton
 **/
public class MotionConstraints {

  private final double vCruise;
  private final double aMax;
  private final double startVelocity;
  private final double endVelocity;
  private final boolean isBackwards;

  /**
   * Constraints used to calculate PathData
   *
   * @param vCruise - max velocity
   * @param aMax - max acceleration
   * @param startVelocity - starting velocity
   * @param endVelocity - ending velocity
   * @param isBackwards - whether or not the path is backwards
   */
  public MotionConstraints(double vCruise, double aMax, double startVelocity, double endVelocity,
      boolean isBackwards) {
    this.vCruise = vCruise;
    this.aMax = aMax;
    this.startVelocity = startVelocity;
    this.endVelocity = endVelocity;
    this.isBackwards = isBackwards;
  }

  public MotionConstraints() {
    this(1, 0.5, 0, 0, false);
  }

  public double getvCruise() {
    return vCruise;
  }

  public double getaMax() {
    return aMax;
  }

  public double getStartVelocity() {
    return startVelocity;
  }

  public double getEndVelocity() {
    return endVelocity;
  }

  public boolean isBackwards() {
    return isBackwards;
  }
}
