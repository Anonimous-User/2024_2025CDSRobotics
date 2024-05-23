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

package frc.robot.subsystems.drive;

// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CANcoder/src/main/java/frc/robot/Robot.java#L42
import static frc.robot.subsystems.drive.DriveConstants.REDUCE_SPARKMAX_CAN_BUS_UTILIZATION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.team6328.util.SparkMaxBurnManager;
import frc.lib.team6328.util.SparkMaxPeriodicFrameConfig;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(Module.ModuleConfig config) {
    driveSparkMax = new CANSparkMax(config.driveCAN(), MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(config.turnCAN(), MotorType.kBrushless);
    turnAbsoluteEncoder = new CANcoder(config.absoluteEncoderCAN());
    absoluteEncoderOffset = config.absoluteEncoderOffset();

    turnAbsolutePosition = turnAbsoluteEncoder.getAbsolutePosition();

    // Reduce the update frequency of the CANCoder Absolute position encoder
    // to avoid saturating CAN bus
    final double updateFrequencyHz = 50; // minimium=4, maximum=1000, disabled=0
    final double timeoutSeconds = 0.050;
    turnAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(updateFrequencyHz, timeoutSeconds);
    turnAbsoluteEncoder.optimizeBusUtilization();

    if (SparkMaxBurnManager.shouldBurn()) {
      driveSparkMax.restoreFactoryDefaults();
      turnSparkMax.restoreFactoryDefaults();
    }

    driveSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    turnSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      if (REDUCE_SPARKMAX_CAN_BUS_UTILIZATION) {
        SparkMaxPeriodicFrameConfig.configNotLeader(driveSparkMax);
        SparkMaxPeriodicFrameConfig.configNotLeader(turnSparkMax);
      }

      // Invert the turning encoder, since the output shaft rotates in the opposite direction of
      // the steering motor in the MAXSwerve Module.
      turnSparkMax.setInverted(config.turnMotorInverted());

      driveSparkMax.setSmartCurrentLimit(50);
      turnSparkMax.setSmartCurrentLimit(20);
      driveSparkMax.enableVoltageCompensation(12.0);
      turnSparkMax.enableVoltageCompensation(12.0);

      driveEncoder.setPosition(0.0);
      driveEncoder.setMeasurementPeriod(10);
      driveEncoder.setAverageDepth(2);

      turnRelativeEncoder.setPosition(0.0);
      turnRelativeEncoder.setMeasurementPeriod(10);
      turnRelativeEncoder.setAverageDepth(2);
    }

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    {
      // NOTE: This is coded this way because MechanicalAdvantage was updating
      // to 1000.0/250.0 = 4ms, but the normal default is 20ms and it kept
      // resulting and dashboard error logs. Not sure what the ideal value is
      // so left it coded like this to investigate later.
      double motorPositionUpdatePeriodMs = 1000.0 / DriveConstants.ODOMETRY_FREQUENCY;
      if (motorPositionUpdatePeriodMs < 20.0) {
        motorPositionUpdatePeriodMs = 20.0;
      }
      if (REDUCE_SPARKMAX_CAN_BUS_UTILIZATION) {
        driveSparkMax.setPeriodicFramePeriod(
            CANSparkLowLevel.PeriodicFrame.kStatus2, (int) motorPositionUpdatePeriodMs);
        turnSparkMax.setPeriodicFramePeriod(
            CANSparkLowLevel.PeriodicFrame.kStatus2, (int) motorPositionUpdatePeriodMs);
      }
    }

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = driveEncoder.getPosition();
                  if (driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    if (SparkMaxBurnManager.shouldBurn()) {
      driveSparkMax.burnFlash();
      turnSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition())
            / DriveConstants.Mk4iReductions.L2.reduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / DriveConstants.Mk4iReductions.L2.reduction;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    // https://github.com/team467/Robot-Code/blob/4d8c30f51ca07885f193a06e084c7c5d9397e32b/src/main/java/frc/robot/subsystems/drive/ModuleIOSparkMAX.java#L101
    // Should we update the turnEncoder based on absolute if not moving?

    // Keep the raw value so we can re-zero if needed
    inputs.rawTurnAbsolutePosition = turnAbsolutePosition.getValueAsDouble();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(inputs.rawTurnAbsolutePosition).minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / DriveConstants.Mk4iReductions.TURN.reduction);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / DriveConstants.Mk4iReductions.TURN.reduction;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) ->
                    Units.rotationsToRadians(value) / DriveConstants.Mk4iReductions.L2.reduction)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRotations(value / DriveConstants.Mk4iReductions.TURN.reduction))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
