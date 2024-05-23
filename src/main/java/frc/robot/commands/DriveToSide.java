package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveToSide extends Command {
  private Drive drive;
  private Timer m_timer;

  public DriveToSide(Drive drive) {
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
    drive.runVelocity(getAllianceChassisSpeeds(-3, -3, 0));
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
    return m_timer.get() > 2;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(getAllianceChassisSpeeds(0, 0, 0));
  }
}
