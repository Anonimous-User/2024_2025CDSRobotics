// Copyright 2021-2024
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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import java.util.OptionalDouble;
import java.util.Queue;

// https://github.com/FRC1257/2024-Robot/blob/1b9c5ed614e1dfac3f83fd12c76cb9d2afb8a773/src/main/java/frc/robot/subsystems/drive/GyroIOReal.java#L17
// https://github.com/SciBorgs/Crescendo-2024/blob/5f63709591bcc00eb6c022d8ba649c98286d0214/src/main/java/org/sciborgs1155/robot/drive/GyroIO.java#L27
// https://github.com/team4099/ChargedUp-2023/blob/d114a9a0cd8de83d438df424b74adf7844050354/src/main/kotlin/com/team4099/robot2023/subsystems/drivetrain/gyro/GyroIONavx.kt#L3
// https://github.com/FRC3636/bunnybots-2023/blob/b717093aeadab5da70fea4af57bef8f4a3be9e17/src/main/java/frc/robot/subsystems/drivetrain/GyroIO.kt#L3
// **
// https://github.com/FRC1257/2024-Robot/blob/871dda75368399f219b66e7020c9641f98b97829/src/main/java/frc/robot/subsystems/drive/GyroIOReal.java#L17
// * SPI
// https://github.com/DevilBotz2876/SwerveDrive2024/blob/f833b780978385e20f0e6002bb6c8797cceae0b5/src/main/java/bhs/devilbotz/subsystems/GyroIONAVX.java#L12

// https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/
//   -
// https://pdocs.kauailabs.com/navx-mxp/installation/orientation-2/
// https://pdocs.kauailabs.com/navx-mxp/guidance/best-practices/
//   Plan for RoboRIO Brownouts - Use a USB cable

public class GyroIONavX2 implements GyroIO {
  // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
  // https://pdocs.kauailabs.com/navx-mxp/guidance/selecting-an-interface/
  static final byte NAVX_DEFAULT_UPDATE_RATE_HZ = 60;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP, NAVX_DEFAULT_UPDATE_RATE_HZ);
  // private final AHRS gyro = new AHRS(SerialPort.Port.kUSB2, AHRS.SerialDataType.kProcessedData,
  // NAVX_DEFAULT_UPDATE_RATE_HZ);

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX2() {
    gyro.reset();
    gyro.resetDisplacement();
    // gyro.zeroYaw();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = gyro.isConnected();
                  if (valid) {
                    return OptionalDouble.of(gyro.getYaw());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(getYawAngle());
    inputs.rollPosition = Rotation2d.fromDegrees(getRollAngle());
    inputs.pitchPosition = Rotation2d.fromDegrees(getPitchAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRawGyroZ());
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    yawPositionQueue.clear();
    yawTimestampQueue.clear();
  }

  private double getYawAngle() {
    return -gyro.getYaw();
  }

  private double getRollAngle() {
    return gyro.getRoll();
  }

  private double getPitchAngle() {
    return gyro.getPitch();
  }
}
