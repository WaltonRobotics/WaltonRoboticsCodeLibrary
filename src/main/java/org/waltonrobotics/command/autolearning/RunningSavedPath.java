package org.waltonrobotics.command.autolearning;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.metadata.PathData;
import org.waltonrobotics.motion.Path;
import org.waltonrobotics.motion.SavePath;

//TODO test this
public final class RunningSavedPath extends SimpleMotion {

  private RunningSavedPath(double maxVelocity, double maxAcceleration, double inverseScale,
      List<PathData> movement) {
    super(SavePath.createPathData(maxVelocity, maxAcceleration, movement).scaleTime(inverseScale));
  }

  private RunningSavedPath(Path savedPath) {
    super(savedPath);
  }

  private RunningSavedPath(double maxVelocity, double maxAcceleration, List<PathData> movement) {
    this(maxVelocity, maxAcceleration, 1.0, movement);
  }

  private RunningSavedPath(double maxVelocity, double maxAcceleration, double inverseScale,
      PathData... movement) {
    this(maxVelocity, maxAcceleration, inverseScale, Arrays.asList(movement));
  }

  private RunningSavedPath(double maxVelocity, double maxAcceleration, PathData... movement) {
    this(maxVelocity, maxAcceleration, 1.0, Arrays.asList(movement));
  }

  public static RunningSavedPath loadSavedPath(String filePath) throws IOException {
    return loadSavedPath(filePath, 1.0);
  }

  public static RunningSavedPath loadSavedPath(String filePath, double speedIncrease)
      throws IOException {
    SavePath path = SavePath.createPathData(Path.loadPath(filePath));

    path = path.scaleTime(speedIncrease);

    return new RunningSavedPath(path);
  }

  public static RunningSavedPath loadSavedPath(LearnPath learnedPath) {
    return loadSavedPath(learnedPath, 1.0);
  }

  public static RunningSavedPath loadSavedPath(LearnPath learnedPath, double speedIncrease) {
    return new RunningSavedPath(
        learnedPath.getMaxVelocity(),
        learnedPath.getMaxAcceleration(),
        speedIncrease, learnedPath.getMoments()
    );
  }


}


