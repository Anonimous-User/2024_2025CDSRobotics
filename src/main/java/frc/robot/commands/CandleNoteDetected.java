package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.sensors.NoteSensor;

public class CandleNoteDetected extends Command {
  private Timer m_timer;
  private Candle candle;
  private NoteSensor noteSensor;

  public CandleNoteDetected(Candle c, NoteSensor noteSensor) {
    candle = c;
    this.noteSensor = noteSensor;
  }

  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.start();
  }

  @Override
  public void execute() {
    candle.green();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > 10 || !noteSensor.noteSensed();
  }

  @Override
  public void end(boolean interrupted) {
    candle.orange();
  }
}
