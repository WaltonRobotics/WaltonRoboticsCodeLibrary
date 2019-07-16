package org.waltonrobotics.command.autolearning;

import java.io.IOException;
import org.waltonrobotics.command.SimpleMotion;
import org.waltonrobotics.motion.Path;

public class PathRunner extends SimpleMotion {

  public PathRunner(Path path) {
    super(path);
  }

  public static PathRunner learnedPath(PathLearner pathLearner) {
    return new PathRunner(pathLearner.getMotion());
  }


  public static PathRunner learnedPath(String fileName) throws IOException {
    return new PathRunner(Path.loadPath(fileName));
  }
}
