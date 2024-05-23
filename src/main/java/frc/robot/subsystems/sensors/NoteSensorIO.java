package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface NoteSensorIO {
  @AutoLog
  public static class NoteSensorIOInputs {
    public boolean rightIntakeSensorActive = false;
    public boolean leftIntakeSensorActive = false;
  }

  /** Update inputs */
  default void updateInputs(NoteSensorIO.NoteSensorIOInputs inputs) {}
}
