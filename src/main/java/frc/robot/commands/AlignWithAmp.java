package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.LimelightWithBotPose;

public class AlignWithAmp extends Command {

  private final Drive m_driveTrain;
  LimelightWithBotPose limelightName;
  double tx, rx, ta;
  int caseNumber = 1;

  public AlignWithAmp(Drive drive, LimelightWithBotPose limelight_name) {
    m_driveTrain = drive;
    addRequirements(m_driveTrain);
    limelightName = limelight_name;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private ChassisSpeeds getAllianceChassisSpeeds(double x, double y, double omega) {
    boolean onRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    if (onRed) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          -x, -y, omega, m_driveTrain.getRotation().plus(new Rotation2d(Math.PI)));
    } else {
      return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_driveTrain.getRotation());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean targetDetected = limelightName.hasTargets();
    SmartDashboard.putBoolean("detected", targetDetected);

    boolean initialized = limelightName.isInitialized();
    if (targetDetected && initialized) {

      Pose3d botPose = limelightName.botPose3D();

      tx = limelightName.x();
      ta = limelightName.targetArea();
      rx = botPose.getRotation().getY() * 45;
      SmartDashboard.putNumber("X", limelightName.x());
      SmartDashboard.putNumber("A", limelightName.targetArea());
      SmartDashboard.putNumber("rX", botPose.getRotation().getY() * 45);
      if (targetDetected) {
        switch (caseNumber) {
          case 1: // rotate to face april tag, use rx
            if (rx > 5) {
              SmartDashboard.putString("driving", "turning right");
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, 3));
            } else if (rx < -5) {
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, -3));
              SmartDashboard.putString("driving", "turning left");
            } else if (rx > -5 && rx < 5) {
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, 0));
              caseNumber = 2;
            }
            break;
          case 2: // strafe to be infront of april tag, use tx
            // works for red side, blue side I don't get paid enough
            if (Math.abs(rx) > 5) {
              caseNumber = 1;
              break;
            }
            if (tx > 2) {
              SmartDashboard.putString("driving", "right");
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 3, 0));
            } else if (tx < -2) {
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, -3, 0));

              SmartDashboard.putString("driving", "left");
            } else if (tx > -2 && tx < 2) {
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, 0));

              caseNumber = 3;
            }
            break;
          case 3: // drive to be at right distance, use ta(or something better)
            SmartDashboard.putString("driving", "forward");
            if (Math.abs(rx) > 5) {
              caseNumber = 1;
              break;
            }
            if (Math.abs(tx) > 2) {
              caseNumber = 2;
              break;
            }
            if (ta < 1.3) { // play with this value
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(3, 0, 0));

            } else {
              m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, 0));

              caseNumber = 1; // goes back to check everything's right
            }
            break;
        }
      } else {
        m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, 0));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { // culd use LED to show it's ended
    m_driveTrain.runVelocity(getAllianceChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(tx)<1 && Math.abs(rx)<1 && ta>5){
    //     return true;
    // }
    return false;
  }
}
