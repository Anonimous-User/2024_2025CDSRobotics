// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Source: https://github.com/Mechanical-Advantage/RobotCode2023

package frc.lib.team6328.util;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

/** Preset configurations for Spark Max periodic frame rates. */
public class SparkMaxPeriodicFrameConfig {
  // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
  //  Status0 - 10ms [6: appliedOutput, faults, stickyFaults, isFollower]
  //  Status1 - 20ms [9: motorVelocity, motorTemperature, motorVoltage, motorCurrent]
  //  Status2 - 20ms [4: motorPosition]
  //  Status3 - 50ms [10: analogSensorVoltage, analogSensorVelocity, analogSensorPosition]
  //  Status4 - 20ms [8: alternateEncoderVelocity, alternateEncoderPosition]
  //  Status5 - 200ms [6: dutyCycleAbsoluteEncoderPosition, dutyCycleAbsoluteEncoderAngle]
  //  Status6 - 200ms [6: dutyCycleAbsoluteEncoderVelocity, dueyCycleAbsoluteEncoderFrequency]
  //  Status7 - 250ms

  public static void configNotLeader(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configLeaderFollower(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }
}
