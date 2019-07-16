package org.waltonrobotics.command.autolearning;

import edu.wpi.first.wpilibj.command.Command;
import java.util.AbstractList;
import java.util.HashMap;
import java.util.function.ToDoubleFunction;
import java.util.stream.DoubleStream;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.command.SimpleLine;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.metadata.ErrorVector;

public class ConstantsFinder extends Command {

  private static AbstractDrivetrain drivetrain;
  private final int lineDistance;
  private final double errorTolerance;
  private SimpleLine driveStraight;
  private HashMap<String, String> pidConstants;
  private AbstractList<ErrorVector> errorVectors;
  private boolean isBackwards = false;
  private int currentPath;

  public ConstantsFinder(int lineDistance, double errorTolerance) {
    this.lineDistance = lineDistance;
    this.errorTolerance = errorTolerance;
    driveStraight = SimpleLine.lineWithDistance(lineDistance);

    drivetrain = SimpleMotion.getDrivetrain();
    requires(drivetrain);
  }

  public ConstantsFinder(double errorTolerance) {
    this(1, errorTolerance);
  }

  public ConstantsFinder(int lineDistance) {
    this(lineDistance, 0.01);
  }

  public ConstantsFinder() {
    this(1, 0.01);
  }

  private void populatePIDConstants() {
    pidConstants.put("kV", String.valueOf(0));
    pidConstants.put("kAcc", String.valueOf(0));
    pidConstants.put("kA", String.valueOf(0));
    pidConstants.put("kS", String.valueOf(0));
    pidConstants.put("kAng", String.valueOf(0));
    pidConstants.put("kL", String.valueOf(0));
    pidConstants.put("iLag", String.valueOf(0));
    pidConstants.put("iAng", String.valueOf(0));

  }

  @Override
  protected void initialize() {
    populatePIDConstants();
    currentPath = drivetrain.getPathNumber();
  }

  @Override
  protected void execute() {
    if (driveStraight.isFinished()) {
      currentPath = drivetrain.getPathNumber();
      errorVectors = drivetrain.getMotionLogger().getErrorVectorList(currentPath);
      if (!errorsWithinTolerance()) {
        guesstimateConstants();
        driveStraight = SimpleLine.lineWithDistance(isBackwards, lineDistance);
        isBackwards = !isBackwards;
        driveStraight.start();
      }
    }
  }

  @Override
  protected void end() {

  }

  public double normalizeSumSquares(AbstractList<ErrorVector> errorVectors) {
    return StrictMath.pow(normalizeSum(errorVectors, ErrorVector::getLag), 2.0) +
        StrictMath.pow(normalizeSum(errorVectors, ErrorVector::getAngle), 2.0) +
        StrictMath.pow(normalizeSum(errorVectors, ErrorVector::getXTrack), 2.0);
  }

  public double normalizeSum(AbstractList<ErrorVector> errorVectors,
      ToDoubleFunction<ErrorVector> toDoubleFunction) {
    DoubleStream doubleStream = errorVectors.parallelStream().mapToDouble(toDoubleFunction);
    double min = doubleStream.min().orElse(0);
    double max = doubleStream.max().orElse(0);
    return doubleStream.map(v -> (v - min) / (max - min)).sum();
  }

  @Override
  protected void interrupted() {
    end();
  }

  @Override
  protected boolean isFinished() {
    return errorsWithinTolerance();
  }

  private void guesstimateConstants() {

//    drivetrain.getDrivetrainProperties().saveProperties();
//    drivetrain.getDrivetrainProperties().savePIDConstants(pidConstants);
//    drivetrain.getDrivetrainProperties().saveToFile();
  }

  private boolean errorsWithinTolerance() {
    return normalizeSumSquares(errorVectors) <= errorTolerance;
//    ErrorVector squareSumErrors = squareSumErrors();
//    return squareSumErrors.getLag() + squareSumErrors.getXTrack() + squareSumErrors.getAngle() <= errorTolerance;
  }

//  private ErrorVector squareSumErrors() {
//    double squareSumLag = 0;
//    double squareSumXTrack = 0;
//    double squareSumAngle = 0;
//
//    for (ErrorVector normalizedErrorVector : normalizeErrorVectors()) {
//      squareSumLag += Math.pow(normalizedErrorVector.getLag(), 2);
//      squareSumXTrack += Math.pow(normalizedErrorVector.getXTrack(), 2);
//      squareSumAngle += Math.pow(normalizedErrorVector.getAngle(), 2);
//    }
//
//    return new ErrorVector(squareSumLag, squareSumXTrack, squareSumAngle);
//  }
//
//  private AbstractList<ErrorVector> normalizeErrorVectors() {
//    ErrorVector maxErrors = getMaxErrors();
//    ErrorVector minErrors = getMinErrors();
//    AbstractList<ErrorVector> normalizedErrorVectors = new LinkedList<>();
//
//    for (ErrorVector errorVector : errorVectors) {
//      double normalizedLag = (errorVector.getLag() - minErrors.getLag()) / maxErrors.getLag();
//      double normalizedXTrack = (errorVector.getXTrack() - minErrors.getXTrack()) / maxErrors.getXTrack();
//      double normalizedAngle = (errorVector.getAngle() - minErrors.getAngle()) / maxErrors.getAngle();
//      normalizedErrorVectors.add(new ErrorVector(normalizedLag, normalizedXTrack, normalizedAngle));
//    }
//
//    return normalizedErrorVectors;
//  }
//
//  private ErrorVector getMaxErrors() {
//    HashMap<String, AbstractList<Double>> splitErrors = splitErrors();
//    double lag = Collections.max(splitErrors.get("lags"));
//    double xTrack = Collections.max(splitErrors.get("xTracks"));
//    double angle = Collections.max(splitErrors.get("angles"));
//    return new ErrorVector(lag, xTrack, angle);
//  }
//
//  private ErrorVector getMinErrors() {
//    HashMap<String, AbstractList<Double>> splitErrors = splitErrors();
//    double lag = Collections.min(splitErrors.get("lags"));
//    double xTrack = Collections.min(splitErrors.get("xTracks"));
//    double angle = Collections.min(splitErrors.get("angles"));
//    return new ErrorVector(lag, xTrack, angle);
//  }
//
//  private HashMap<String, AbstractList<Double>> splitErrors() {
//    HashMap<String, AbstractList<Double>> splitErrors = new HashMap<>();
//    AbstractList<Double> lags = new LinkedList<>();
//    AbstractList<Double> xTracks = new LinkedList<>();
//    AbstractList<Double> angles = new LinkedList<>();
//
//    for (ErrorVector errorVector : errorVectors) {
//      lags.add(errorVector.getLag());
//      xTracks.add(errorVector.getXTrack());
//      angles.add(errorVector.getAngle());
//    }
//
//    splitErrors.put("lags", lags);
//    splitErrors.put("xTracks", xTracks);
//    splitErrors.put("angles", angles);
//    return splitErrors;
//  }
}
