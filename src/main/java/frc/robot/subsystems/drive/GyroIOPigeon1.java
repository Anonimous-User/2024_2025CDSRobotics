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

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon1 */
public class GyroIOPigeon1 implements GyroIO {
  private final PigeonIMU pigeon;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private double previousYaw;
  private double previousTimestamp;

  public GyroIOPigeon1(int pigeonCAN) {
    pigeon = new PigeonIMU(pigeonCAN);
    pigeon.configFactoryDefault();
    pigeon.setYaw(0.0);
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = pigeon.getState() == PigeonIMU.PigeonState.Ready;
                  if (valid) {
                    return OptionalDouble.of(pigeon.getYaw());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    previousYaw = pigeon.getYaw();
    previousTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.getState() == PigeonIMU.PigeonState.Ready;
    inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw());

    double yawVelocity =
        (pigeon.getYaw() - previousYaw) / (previousTimestamp - Timer.getFPGATimestamp());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity);
    previousYaw = pigeon.getYaw();
    previousTimestamp = Timer.getFPGATimestamp();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
