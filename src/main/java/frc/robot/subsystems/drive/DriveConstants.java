package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.Module.ModulePosition.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Module.ModuleConfig;
import frc.robot.subsystems.drive.Module.ModulePosition;
import frc.robot.util.RobotType;
import java.util.EnumMap;

public class DriveConstants {
  public static final boolean REDUCE_SPARKMAX_CAN_BUS_UTILIZATION = true;
  public static final int PIGEON1_CAN_ID = 40;
  public static final int PIGEON2_CAN_ID = 61;
  public static final String PIGEON2_CANBUS = "*"; // Star is ANY CANivore
  static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  static final double ODOMETRY_FREQUENCY = 250.0;
  static final double TRACK_WIDTH_X = Units.inchesToMeters(23.5);
  static final double TRACK_WIDTH_Y = Units.inchesToMeters(23.5);
  static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  static final double MAX_LINEAR_SPEED = Units.feetToMeters(24.5);
  static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final double BUMPER_WIDTH_X = Units.inchesToMeters(35);
  static final double BUMPER_WIDTH_Y = Units.inchesToMeters(35);

  public static EnumMap<ModulePosition, ModuleConfig> moduleConfigs =
      switch (Constants.currentMode) {
        case REAL -> {
          var configs = new EnumMap<ModulePosition, ModuleConfig>(ModulePosition.class);
          configs.put( // 0.7954...  0.2495
              FRONT_LEFT,
              new ModuleConfig(
                  6,
                  5,
                  33,
                  Rotation2d.fromRotations(0.547).plus(Rotation2d.fromRadians(Math.PI)),
                  true)); // 0.2490234375), true));
          configs.put( // 0.7120.... 0.1711
              FRONT_RIGHT,
              new ModuleConfig(
                  8,
                  7,
                  34,
                  Rotation2d.fromRotations(0.5449).plus(Rotation2d.fromRadians(Math.PI)),
                  true)); // 0.171630859375), true));
          configs.put( // 0.3691... 0.8571
              REAR_LEFT,
              new ModuleConfig(
                  4,
                  3,
                  32,
                  Rotation2d.fromRotations(0.5155).plus(Rotation2d.fromRadians(Math.PI)),
                  true)); // 0.85791015625), true));
          configs.put( // 0.2873... 0.7819
              REAR_RIGHT,
              new ModuleConfig(
                  2,
                  1,
                  31,
                  Rotation2d.fromRotations(0.5107).plus(Rotation2d.fromRadians(Math.PI)),
                  true)); // 0.781005859), true));
          yield configs;
        }
        case SIM, REPLAY -> {
          yield null;
        }
      };

  public enum Mk4iReductions {
    // Gear ratios for SDS MK4i L2: DriveRatio=16.746, TurnRatio=21.428
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }

  // Odometry Constants
  public static final double odometryFrequency =
      switch (RobotType.getRobot()) {
        case ROBOT_SIMBOT -> 50.0;
        case ROBOT_2024C -> 250.0;
      };
}
