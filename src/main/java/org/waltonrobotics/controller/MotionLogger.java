package org.waltonrobotics.controller;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.AbstractList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import org.waltonrobotics.metadata.ErrorVector;
import org.waltonrobotics.metadata.MotionData;

/**
 * This class is used to log MotionData during autonomous
 *
 * @author Russell Newton, WaltonRobotics
 */
public class MotionLogger {

  private final AbstractList<MotionData> motionDataList;
  private final String filePath;

  /**
   * Create this in robotInit() before making the drivetrain
   *
   * @param filePath - Where do you want to save the logs? To save to the roboRIO, use base
   * directory "/home/lvuser/". To save to a thumb drive, use winSCP or similar program to find the
   * right filepath
   */
  public MotionLogger(String filePath) {
    motionDataList = new LinkedList<>();
    this.filePath = filePath;
  }

  /**
   * Create this in robotInit() before making the drivetrain
   *
   * Uses the default "/home/lvuser/" location on the roboRIO to save the logs
   */
  public MotionLogger() {
    this("/home/lvuser/");
  }

  /**
   * This is called in the MotionController to add MotionData to the motionDataList that
   * MotionLogger has
   */
  public final synchronized void addMotionData(MotionData dataAdd) {
    motionDataList.add(dataAdd);
  }

  /**
   * Call this in autonomousInit() to clear the motionDataList
   */
  public final synchronized void initialize() {
    motionDataList.clear();
  }

  public final synchronized void writeMotionDataCSV() {
    writeMotionDataCSV(false);
  }

  /**
   * Saves data to a .csv file with the name in "yyyy-MM-dd hh-mm-ss" format. Call this in
   * disabledInit() to send the motionDataList to a .csv file.
   */
  public final synchronized void writeMotionDataCSV(boolean clearData) {
    if (motionDataList.isEmpty()) {
      return;
    }
    String fileName = new SimpleDateFormat("yyyy-MM-dd hh-mm-ss").format(new Date());
    File file = new File(filePath + fileName + ".csv");

    StringBuilder sb = new StringBuilder(204800);
    sb.append("Time");
    sb.append(", ");
    sb.append("xActual");
    sb.append(", ");
    sb.append("yActual");
    sb.append(", ");
    sb.append("angleActual");
    sb.append(", ");
    sb.append("xTarget");
    sb.append(", ");
    sb.append("yTarget");
    sb.append(", ");
    sb.append("angleTarget");
    sb.append(", ");
    sb.append("XTE");
    sb.append(", ");
    sb.append("lagE");
    sb.append(", ");
    sb.append("angleE");
    sb.append(", ");
    sb.append("pLeft");
    sb.append(", ");
    sb.append("pRight");
    sb.append(", ");
    sb.append("pathNumber");
    sb.append(", ");
    sb.append("motionState");
    sb.append('\n');

    for (MotionData data : motionDataList) {
      sb.append(data.getPowers().getTime());
      sb.append(", ");
      sb.append(data.getActualPose().getX());
      sb.append(", ");
      sb.append(data.getActualPose().getY());
      sb.append(", ");
      sb.append(data.getActualPose().getAngle());
      sb.append(", ");
      sb.append(data.getTargetPose().getX());
      sb.append(", ");
      sb.append(data.getTargetPose().getY());
      sb.append(", ");
      sb.append(data.getTargetPose().getAngle());
      sb.append(", ");
      sb.append(data.getError().getXTrack());
      sb.append(", ");
      sb.append(data.getError().getLag());
      sb.append(", ");
      sb.append(data.getError().getAngle());
      sb.append(", ");
      sb.append(data.getPowers().getLeft());
      sb.append(", ");
      sb.append(data.getPowers().getRight());
      sb.append(", ");
      sb.append(data.getPathNumber());
      sb.append(", ");
      sb.append(data.getCurrentMotionState());
      sb.append('\n');
    }

    try (BufferedWriter bf = new BufferedWriter(new FileWriter(file), 32768)) {
      System.out.println("File " + fileName + " has been made!");
      bf.write(sb.toString());
//			pw.flush();
//			pw.close();

      if (clearData) {
        motionDataList.clear();
      }
    } catch (FileNotFoundException e) {
      System.out.println("There is no file at " + file);
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public AbstractList<ErrorVector> getErrorVectorList(int pathNumber) {
    AbstractList<ErrorVector> errorVectors = new LinkedList<>();
    for (MotionData data : motionDataList) {
      if (data.getPathNumber() == pathNumber) {
        errorVectors.add(data.getError());
      }
    }
    return errorVectors;
  }

  public List<ErrorVector> getErrorVectorList() {
    return motionDataList.stream().map(MotionData::getError).collect(Collectors.toList());
  }

  @Override
  public String toString() {
    return "MotionLogger{" +
        "motionDataList=" + motionDataList +
        ", filePath='" + filePath + '\'' +
        '}';
  }
}
