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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double CONTROLLER_DEADBAND = 0.2;

  private static final double speedAdjustment = 1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier relativeSupplier,
      BooleanSupplier slowSupplier) {
    return Commands.run(
        () -> {
          double controllerX = xSupplier.getAsDouble();
          double controllerY = ySupplier.getAsDouble();
          double controllerOmega = omegaSupplier.getAsDouble();
          boolean robotRelative = relativeSupplier.getAsBoolean();
          boolean moveSlowly = slowSupplier.getAsBoolean();
          double controllerDeadband = CONTROLLER_DEADBAND;

          double slowMultiplier = 1.0;
          if (moveSlowly) {
            slowMultiplier = 0.4;
          }

          Translation2d linearVelocity = calcLinearVelocity(controllerX, controllerY);
          double omega = MathUtil.applyDeadband(controllerOmega, controllerDeadband);
          omega = Math.copySign(omega * omega, omega);

          final double maxAngularVelocity = drive.getMaxAngularSpeedRadPerSec();
          ChassisSpeeds updatedSpeeds = null;
          if (robotRelative) {
            updatedSpeeds =
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * slowMultiplier,
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * slowMultiplier,
                    omega * maxAngularVelocity * slowMultiplier);
          } else {
            if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
            }
            updatedSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * slowMultiplier,
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * slowMultiplier,
                    omega * maxAngularVelocity * slowMultiplier,
                    drive.getPose().getRotation());
          }

          drive.runVelocity(updatedSpeeds);
        },
        drive);
  }

  private static Translation2d calcLinearVelocity(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), CONTROLLER_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square magnitude
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calculate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }

  public static double scaleSpeed(double speed) {
    return speed * speedAdjustment;
  }
}
