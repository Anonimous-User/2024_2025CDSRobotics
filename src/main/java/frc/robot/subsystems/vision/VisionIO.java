// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * public Pose2d[] _poses = new Pose2d[HardwareConstants.NUMBER_OF_CAMERAS]; public double[]
 * _timestamps = new double[HardwareConstants.NUMBER_OF_CAMERAS]; public boolean[] _hasPose = new
 * boolean[HardwareConstants.NUMBER_OF_CAMERAS];
 */

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  public static class VisionIOInputs implements LoggableInputs {
    public double captureTimestamp = 0.0;
    public double[] cornerX = new double[] {};
    public double[] cornerY = new double[] {};
    public boolean simpleValid = false;
    public double simpleAngle = 0.0;

    public void toLog(LogTable table) {
      table.put("CaptureTimestamp", captureTimestamp);
      table.put("CornerX", cornerX);
      table.put("CornerY", cornerY);
      table.put("SimpleValid", simpleValid);
      table.put("SimpleAngle", simpleAngle);
    }

    public void fromLog(LogTable table) {
      captureTimestamp = table.get("CaptureTimestamp", captureTimestamp);
      cornerX = table.get("CornerX", cornerX);
      cornerY = table.get("CornerY", cornerY);
      simpleValid = table.get("SimpleValid", simpleValid);
      simpleAngle = table.get("SimpleAngle", simpleAngle);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Enabled or disabled vision LEDs. */
  public default void setLeds(boolean enabled) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}
