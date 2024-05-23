package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.PIDGains;

public class ArmConstants {
  public static final double ABSOLUTE_FRONT_FLOOR_ROTATIONS = 0.1073312276832807;
  public static final double ABSOLUTE_REAR_FLOOR_ROTATIONS = 0.6097679652441992;
  public static final double ENCODER_FRONT_FLOOR_ROTATIONS = 2.77580883026123;
  public static final double ENCODER_REAR_FLOOR_ROTATIONS = 1.129279558670044;

  public static final int kArmFollowerCanId = 12;
  public static final int kArmLeaderCanId = 11;

  public static final int kCurrentLimit = 40;
  public static final double kVoltageCompenstation = 12.0;

  public static final double kSoftLimitReverse = -1.65;
  public static final double kSoftLimitForward = 0;

  // TODO: What is this value?
  public static final double kArmGearRatio = (1.0 / 25.0) * (28.0 / 50.0) * (16.0 / 64.0);
  public static final double kPositionFactor =
      kArmGearRatio
          * 2.0
          * Math.PI; // multiply SM value by this number and get arm position in radians
  public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;

  // TODO: Disabled ArmFeedForward - their arm is different physically
  //  public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
  //
  //  public static final double kArmZeroCosineOffset =
  //      1.342; // radians to add to converted arm position to get real-world arm position (starts
  // at
  //  // ~76.9deg angle)
  //  public static final ArmFeedforward kArmFeedforward =
  //      new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
  public static final PIDGains kArmPositionGains = new PIDGains(0.03, 0.0, 0.0);
  public static final TrapezoidProfile.Constraints kArmMotionConstraint =
      new TrapezoidProfile.Constraints(0.02, 0.08);

  public static final double kHomePosition = 0.0;
  public static final double kScoringPosition = 0.0;
  public static final double kIntakePosition = -1.17;
}
