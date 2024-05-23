// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Original Source: https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.Constants.Mode;
import java.util.Map;

public final class RobotType {
  private static final Type robot = Type.ROBOT_2024C;
  public static boolean invalidRobotAlertSent = false;

  public static Type getRobot() {
    if (!disableHAL && RobotBase.isReal()) {
      if (robot == Type.ROBOT_SIMBOT) { // Invalid robot selected
        if (!invalidRobotAlertSent) {
          new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
              .set(true);
          invalidRobotAlertSent = true;
        }
        return Type.ROBOT_2024C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2024C:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<Type, String> logFolders = Map.of(Type.ROBOT_2024C, "/media/sda2/");

  public enum Type {
    ROBOT_2024C,
    ROBOT_SIMBOT
  }

  // Function to disable HAL interaction when running without native libs
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == Type.ROBOT_SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}
