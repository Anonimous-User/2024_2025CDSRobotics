// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Set this up so I could toggle to false when running on blocks and have the Gyro simulated
  // and not worry about the arm moving around. It should be TRUE by default unless debugging
  // on blocks.
  public static final boolean RUNNING_FULL_FEATURED_ROBOT = true;

  // Need to choose if we have a camera or logging enabled. Ideally,
  // we move USB camera to OrangePi.
  public static final boolean USE_USB_CAMERA_ON_RIO2 = true;

  public static final boolean USE_PATH_PLANNING_STUFF = false;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = true;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
