package org.waltonrobotics.controller;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.Timer;
import org.waltonrobotics.metadata.CameraData;

public class CameraReader {

  private final SerialPort serialPort;
  private final String[] cmdCommands = {
      "streamoff",
      "setmapping2 YUYV 640 480 30.0 WaltonRobotics DeepSpace",
      "setpar serout All",
      "setpar serlog USB",
      "streamon"
  };

  private boolean isRunning = false;
  private CameraData cameraData = new CameraData();

  public CameraReader(int baudRate, Port portType) {
    SerialPort serialPort;
    try {

      serialPort = new SerialPort(baudRate, portType);
      serialPort.setWriteBufferMode(WriteBufferMode.kFlushWhenFull);
      serialPort.enableTermination();
      serialPort.setReadBufferSize(18);
      System.out.println("Camera found");
    } catch (UncleanStatusException exception) {
      serialPort = null;
      System.out.println("Could not find camera");
      exception.printStackTrace();
    }

    this.serialPort = serialPort;
  }

  public CameraReader(int baudRate) {
    this(baudRate, Port.kUSB);
  }

  public CameraReader(Port port) {
    this(115200, port);
  }

  public CameraReader() {
    this(115200);
  }

  public String readRaw() {
    return serialPort.readString();
  }

  public void writeCommand(String command) {
    serialPort.writeString(command + '\n');
  }

  public CameraData getCameraData() {
    return cameraData;
  }

  public synchronized void run() {
    if (serialPort != null) {
      if (!isRunning) {
        startCollecting();
      }
      if (isRunning) {

        String data = readRaw().trim();

//    System.out.println(data.matches("(F)|(\\d{2,})"));
        System.out.println(data);

        if (data.length() > 0) {
//    if (data.length() > 17) {
          if (data.matches("^[xX]\\d{3}[yY]\\d{3}[zZ]\\d{3}[aA]\\d{3}N\\d+$")) {
            int x = Integer.parseUnsignedInt(data.substring(1, 4)) *
                ((data.charAt(0) == 'X') ? 1 : -1);
            int y = Integer.parseUnsignedInt(data.substring(5, 8)) *
                ((data.charAt(4) == 'Y') ? 1 : -1);
            int z = Integer.parseUnsignedInt(data.substring(9, 12)) *
                ((data.charAt(8) == 'Z') ? 1 : -1);
            int angle = Integer.parseUnsignedInt(data.substring(13, 16)) *
                ((data.charAt(12) == 'A') ? 1 : -1);
            int numberOfTargets = Integer.parseUnsignedInt(data.substring(17));
            double t = Timer.getFPGATimestamp() - 0.052;

            this.cameraData = new CameraData(x / 100.0, y / 100.0, z, angle, numberOfTargets, t);
          } else if (data.matches("^FN\\d+$")) {
            int numberOfTargets = Integer.parseUnsignedInt(data.substring(2));
            this.cameraData = new CameraData(numberOfTargets);
          } else {
            this.cameraData = new CameraData();
          }
        } else {
          System.out.println("No camera data");
        }
      }
    }
  }

  public void startCollecting() {
    if (serialPort != null) {

      for (String cmdCommand : cmdCommands) {
        writeCommand(cmdCommand);
      }

      System.out.println("Started collecting");

      isRunning = true;
    }
  }
}
