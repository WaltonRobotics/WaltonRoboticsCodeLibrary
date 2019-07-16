package org.waltonrobotics.motion;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import org.waltonrobotics.command.autolearning.LearnPath;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.metadata.State;

public final class SavePath extends Path {

  private SavePath(double vCruise, double aMax,
      Collection<PathData> pathData) {
    super(vCruise, aMax, false,
        pathData.stream().map(PathData::getCenterPose).collect(Collectors.toList()));

    getPathData().clear();
    getPathData().addAll(pathData);
  }

  public static SavePath createPathData(double maxVelocity, double maxAcceleration,
      List<PathData> pathData) {
    return new SavePath(maxVelocity, maxAcceleration, pathData);
  }

  public static SavePath createPathData(Path path) {
    return new SavePath(path.getVCruise(), path.getVCruise(), path.getPathData());
  }

  public static SavePath createPathData(LearnPath learnedPath) {
    return new SavePath(learnedPath.getMaxVelocity(), learnedPath.getMaxAcceleration(),
        learnedPath.getMoments());
  }

  public SavePath setMaxVelocity(double maxVelocity) {
    double scale = maxVelocity / getVCruise();

    return scaleTime(scale);
  }

  /**
   * Will scale time to be short the bigger the number. E.i. passing in 3 means that time will go 3
   * time faster meaning that the robot will complete the org.waltonrobotics.motion faster.
   */
  public SavePath scaleTime(double inverseScale) {
    if (inverseScale != 1.0) {
      List<PathData> updatedPathData = getPathData().stream().map(moment ->
          new PathData(
              new State(moment.getLeftState().getLength(),
                  moment.getLeftState().getVelocity() * inverseScale,
                  moment.getLeftState().getAcceleration() * inverseScale),
              new State(moment.getRightState().getLength(),
                  moment.getRightState().getVelocity() * inverseScale,
                  moment.getRightState().getAcceleration() * inverseScale)
              , moment.getCenterPose(),
              moment.getTime() / inverseScale
          )
      ).collect(Collectors.toList());

      getPathData().clear();
      aMax = (getAMax() * inverseScale);
      vCruise = (getVCruise() * inverseScale);

      getPathData().addAll(updatedPathData);
    }
    return this;
  }


}
