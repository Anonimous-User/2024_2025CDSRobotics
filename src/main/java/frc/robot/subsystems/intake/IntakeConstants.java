// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.util.PIDGains;

public class IntakeConstants {

  public static final int kCanId = 52;
  public static final int kCurrentLimit = 80;
  public static final PIDGains kPositionGains = new PIDGains(0.1, 0, 0.0);
  public static final double kPositionTolerance = 0.5;
  public static final double kRetractDistance = -5.0;

  public static final double kShotFeedTime = 1.0;
}
