// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Original Source: https://github.com/Mechanical-Advantage/RobotCode2023

package frc.lib.team6328.util;

import static frc.robot.Constants.Mode.REAL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import frc.robot.util.RobotType;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

// Utilizes the Waveshare Barcode Scanner Module 1D/2D Codes Barcode, QR Code Reader Onboard Micro
// USB and
// UART Serial Port Directly Plugged into Computer.
// https://www.waveshare.com/w/upload/d/dd/Barcode_Scanner_Module_Setting_Manual_EN.pdf
public class BatteryTracker {
  private static final String batteryNameFile = "/home/lvuser/battery-name.txt";
  private static boolean batteryNameWritten = false;
  private static final List<RobotType.Type> supportedRobots = List.of(RobotType.Type.ROBOT_2024C);
  public static final String defaultName = "0000-000";

  private static final int nameLength = 8;

  // This uses "Command Mode" (Barcode_Scanner_Module_Setting_Manual_EN.pdf, page 21)
  private static final byte[] scanCommand =
      new byte[] {
        0x7e,
        0x00, // Head1
        0x08, // Type=Write
        0x01, // Lens
        0x00,
        0x02, // Start Address of read
        0x01, // Read 1 byte
        (byte) 0xab,
        (byte) 0xcd
      }; // CRC_CCITT checksum == IGNORE Checksum 0xab, 0xcd
  private static final byte[] responsePrefix =
      new byte[] {
        0x02, 0x00, // Head2
        0x00, // Read Success
        0x01, // Lens
        0x00, // Data
        0x33, 0x31
      }; // CRC_CCITT checksum
  private static final int fullResponseLength = responsePrefix.length + nameLength;

  private static String name = defaultName;

  private static final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", Alert.AlertType.WARNING);

  public static void recordBattery(double timeout) {
    System.out.println("[Init] Scanning battery");
    // Logger.recordMetadata("BatteryName", "BAT-" + BatteryTracker.scanBattery(timeout));
  }

  public static void checkSameBattery() {

    // Check for battery alert
    if (Constants.currentMode == REAL
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)) {
      File file = new File(batteryNameFile);
      if (file.exists()) {
        // Read previous battery name
        String previousBatteryName = "";
        try {
          previousBatteryName =
              new String(Files.readAllBytes(Paths.get(batteryNameFile)), StandardCharsets.UTF_8);
        } catch (IOException e) {
          e.printStackTrace();
        }

        if (previousBatteryName.equals(BatteryTracker.getName())) {
          // Same battery, set alert
          sameBatteryAlert.set(true);
        } else {
          // New battery, delete file
          file.delete();
        }
      }
    }
  }

  /**
   * Scans the battery. This should be called before the first loop cycle
   *
   * @param timeout The time to wait before giving up
   */
  public static String scanBattery(double timeout) {
    if (RobotType.getMode() == Constants.Mode.REAL) {
      if (supportedRobots.contains(RobotType.getRobot())) {
        // Only scan on supported robots and in real mode

        // Mechanical Advantage connects to SerialPort.Port.kUSB, below we connect using the
        // UART of the NavX2 passed through the MXP connection to free up our USB port.
        // ie. UART dip switch on NavX2 is OFF to prevent NavX2 from activating UART. The scan
        // command and response is the otherwise the same as Mechanical Advantage.
        //
        // Parameters: Baud rate, Data port (MXP), Data bits, Parity, Stop bits
        try (SerialPort barcodeScanner =
            new SerialPort(
                9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne)) {
          barcodeScanner.setTimeout(timeout);
          barcodeScanner.setWriteBufferSize(scanCommand.length);
          barcodeScanner.setReadBufferSize(fullResponseLength);

          barcodeScanner.write(scanCommand, scanCommand.length);
          byte[] response = barcodeScanner.read(fullResponseLength);

          // Ensure response is correct length
          if (response.length != fullResponseLength) {
            System.out.println(
                "[BatteryTracker] Expected "
                    + fullResponseLength
                    + " bytes from scanner, got "
                    + response.length);
            return name;
          }

          // Ensure response starts with prefix
          for (int i = 0; i < responsePrefix.length; i++) {
            if (response[i] != responsePrefix[i]) {
              System.out.println("[BatteryTracker] Invalid prefix from scanner.  Got data:");
              System.out.println("[BatteryTracker] " + Arrays.toString(response));
              return name;
            }
          }

          // Read name from data
          byte[] batteryNameBytes = new byte[nameLength];
          System.arraycopy(response, responsePrefix.length, batteryNameBytes, 0, nameLength);
          name = new String(batteryNameBytes);
          System.out.println("[BatteryTracker] Scanned battery " + name);

        } catch (Exception e) {
          System.out.println("[BatteryTracker] Exception while trying to scan battery");
          e.printStackTrace();
        }
      }
    }

    return name;
  }

  public static void writeBatteryName() {
    if (!batteryNameWritten) {
      // Write battery name if connected to field
      if (RobotType.getMode() == Constants.Mode.REAL
          && !BatteryTracker.getName().equals(BatteryTracker.defaultName)
          && DriverStation.isFMSAttached()) {
        batteryNameWritten = true;
        try {
          FileWriter fileWriter = new FileWriter(batteryNameFile);
          fileWriter.write(BatteryTracker.getName());
          fileWriter.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }
  }

  /** Returns the name of the last scanned battery. */
  public static String getName() {
    return name;
  }
}
