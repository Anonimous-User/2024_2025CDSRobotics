package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class NoteSensorIODIO implements NoteSensorIO {
  private final DigitalInput rightIntakeSensor;
  private final DigitalInput leftIntakeSensor;

  public NoteSensorIODIO() {
    rightIntakeSensor = new DigitalInput(NoteSensorConstants.RIGHT_INTAKE_SENSOR_DIO);
    leftIntakeSensor = new DigitalInput(NoteSensorConstants.LEFT_INTAKE_SENSOR_DIO);
  }

  public void updateInputs(NoteSensorIO.NoteSensorIOInputs inputs) {
    // The DIO pins return high by default. We are looking for
    // when they sense something which is when it would be false.
    // For code readability, it makes sense then that our Active
    // is the opposite signal of what we read.
    inputs.rightIntakeSensorActive = !rightIntakeSensor.get();
    inputs.leftIntakeSensorActive = !leftIntakeSensor.get();
  }
}
