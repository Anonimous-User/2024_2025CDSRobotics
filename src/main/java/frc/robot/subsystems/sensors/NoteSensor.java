package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class NoteSensor extends SubsystemBase {
  private final NoteSensorIO io;

  private final NoteSensorIOInputsAutoLogged inputs = new NoteSensorIOInputsAutoLogged();

  public NoteSensor(NoteSensorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("NoteSensor", inputs);
  }

  public Command waitForRightSensor() {
    return Commands.waitUntil(() -> noteSensedByRightSensor());
  }

  public Command waitForLeftSensor() {
    return Commands.waitUntil(() -> noteSensedByLeftSensor());
  }

  /**
   * @returns true if there is a note in the intake
   */
  public boolean noteSensedByRightSensor() {
    return inputs.rightIntakeSensorActive;
  }

  public boolean noteSensedByLeftSensor() {
    return inputs.leftIntakeSensorActive;
  }

  public boolean noteSensed() {
    return noteSensedByLeftSensor() || noteSensedByLeftSensor();
  }
}
