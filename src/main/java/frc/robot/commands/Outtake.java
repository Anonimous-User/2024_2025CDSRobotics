package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSimple;

public class Outtake extends Command {
  private Timer m_timer;
  private IntakeSimple intake;

  public Outtake(IntakeSimple in) {
    intake = in;
  }

  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
  }

  @Override
  public void execute() {
    intake.setPower(-0.75);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > 1;
  }

  @Override
  public void end(boolean interrupted) {
    intake.setPower(0);
  }
}
