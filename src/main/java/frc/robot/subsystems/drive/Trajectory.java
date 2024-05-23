package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.team6328.FieldConstants;

public class Trajectory {
  // Starting locations
  public static final Pose2d startingBackOnAmpFace =
      FieldConstants.Subwoofer.ampFaceCorner.transformBy(
          new Transform2d(
              -DriveConstants.BUMPER_WIDTH_X / 2,
              -DriveConstants.BUMPER_WIDTH_Y / 2,
              new Rotation2d(Math.toRadians(180.0)))); // Front Faces Away from Subwoofer
  public static final Pose2d startingBackOnSourceFace =
      FieldConstants.Subwoofer.sourceFaceCorner.transformBy(
          new Transform2d(
              -DriveConstants.BUMPER_WIDTH_X / 2,
              DriveConstants.BUMPER_WIDTH_Y / 2,
              new Rotation2d(Math.toRadians(180.0)))); // Front Faces Away from Subwoofer
  public static final Pose2d startingBackOnCenterFace =
      FieldConstants.Subwoofer.centerFace.transformBy(
          new Transform2d(
              -DriveConstants.BUMPER_WIDTH_X / 2,
              0,
              new Rotation2d(Math.toRadians(180.0)))); // Front Faces Away from Subwoofer
}
