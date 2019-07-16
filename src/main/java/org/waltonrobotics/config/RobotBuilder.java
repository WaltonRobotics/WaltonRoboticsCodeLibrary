package org.waltonrobotics.config;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;
import java.util.Collections;

public class RobotBuilder<T extends RobotConfig> {

  private ArrayList<T> robotConfigs = new ArrayList<>();

  public RobotBuilder(ArrayList<T> robotConfigs) {
    this.robotConfigs = robotConfigs;
  }

  public RobotBuilder(T... robotConfigs) {
    Collections.addAll(this.robotConfigs, robotConfigs);
  }

  public ArrayList<T> getRobotConfigs() {
    return robotConfigs;
  }

  public T getCurrentRobotConfig() {
    return robotConfigs.stream().filter(RobotConfig::isCurrentRobot).findFirst().orElse(null);
  }

  public SendableChooser<T> getSendableList() {
    SendableChooser<T> robotConfigSendableChooser = new SendableChooser<>();
    for (T robotConfig : robotConfigs) {
      robotConfigSendableChooser.addOption(robotConfig.getRobotName(), robotConfig);
    }

    T currentRobot = getCurrentRobotConfig();
    robotConfigSendableChooser.setDefaultOption(currentRobot.getRobotName(), currentRobot);

    return robotConfigSendableChooser;

  }
}
