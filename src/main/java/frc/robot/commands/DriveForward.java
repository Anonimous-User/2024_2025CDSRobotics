package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveForward extends Command {
  private Drive drive;
  private Timer m_timer;

  public DriveForward(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
  }

  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_timer.get() < 2) {
      drive.runVelocity(getAllianceChassisSpeeds(2, 0, 0));
    } else if (m_timer.get() < 4) {
      drive.runVelocity(getAllianceChassisSpeeds(0, 0, 0));
    } else {
      drive.runVelocity(getAllianceChassisSpeeds(-2, 0, 0));
    }
  }

  private ChassisSpeeds getAllianceChassisSpeeds(double x, double y, double omega) {
    boolean onRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    if (onRed) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          -x, -y, omega, drive.getRotation().plus(new Rotation2d(Math.PI)));
    } else {
      return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, drive.getRotation());
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > 6;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(getAllianceChassisSpeeds(0, 0, 0));
  }
}
