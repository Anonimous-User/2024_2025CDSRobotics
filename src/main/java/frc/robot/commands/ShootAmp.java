package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSimple;
import frc.robot.subsystems.shooter.ShooterSimple;

public class ShootAmp extends Command {
  private Timer m_timer;
  private ShooterSimple shooter;
  private IntakeSimple intake;

  public ShootAmp(ShooterSimple shoot, IntakeSimple in) {
    shooter = shoot;
    intake = in;
  }

  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
  }

  @Override
  public void execute() {
    shooter.runShooter();
    intake.setPower(1);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > 1;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopLauncher();
    intake.setPower(0);
  }
}
