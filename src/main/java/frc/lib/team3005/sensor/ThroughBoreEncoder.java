package frc.lib.team3005.sensor;

// Modified from:
// https://github.com/FRC3005/Charged-Up-2023-Public/blob/2b6a7c695e23edebafa27a76cf639a00f6e8a3a6/src/main/java/frc/lib/vendor/sensor/ThroughBoreEncoder.java

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class ThroughBoreEncoder {
  private final DigitalInput m_digitalSource;
  private final DutyCycle m_dutyCycle;
  private final DutyCycleEncoder m_encoder;
  private double m_offset = 0.0;
  private final double m_scalar;
  private final boolean m_inverted;

  public ThroughBoreEncoder(int channel, double offset, double scalar, boolean invert) {
    m_digitalSource = new DigitalInput(channel);
    m_dutyCycle = new DutyCycle(m_digitalSource);
    m_encoder = new DutyCycleEncoder(m_dutyCycle);
    m_offset = offset;
    m_scalar = scalar;
    m_inverted = invert;
  }

  public ThroughBoreEncoder(int channel) {
    this(channel, 0.0, 1.0, false);
  }

  public DigitalSource getDigitalSource() {
    return m_digitalSource;
  }

  public double getPosition() {
    double result;
    if (m_inverted) {
      result = ((1.0 - m_dutyCycle.getOutput()) * m_scalar - m_offset);
    } else {
      result = (m_dutyCycle.getOutput() * m_scalar - m_offset);
    }
    return MathUtil.inputModulus(result, 0, m_scalar);
  }

  public boolean isConnected() {
    return m_encoder.isConnected();
  }

  public double dutyCycle() {
    if (m_inverted) {
      return 1.0 - m_dutyCycle.getOutput();
    }
    return m_dutyCycle.getOutput();
  }

  public double getFrequency() {
    return m_dutyCycle.getFrequency();
  }

  public double getRelativePosition() {
    return m_encoder.get();
  }

  public void setPositionOffset(double offset) {
    m_offset = offset;
  }

  public double getPositionOffset() {
    return m_offset;
  }

  public double getPositionScalar() {
    return m_scalar;
  }

  public boolean getPositionInverted() {
    return m_inverted;
  }

  /**
   * There is a quirk when first starting code that the duty cycle class reads back a frequency that
   * is too low, and all the measurements are off. This function will block until the correct range
   * is found.
   *
   * @return true if range is found
   */
  public boolean waitForInit(double timeoutSeconds) {
    // Values from data sheet plus some margin
    final double expectedMinFrequency = 924.0;
    final double expectedMaxFrequency = 1024.0;
    var initTimer = new Timer();
    initTimer.start();

    if (Robot.isSimulation()) {
      return true;
    }

    while (!initTimer.hasElapsed(timeoutSeconds)) {
      // Through bore encoder frequency should be in a specific range, when
      // outside that range, we know its not working quite right.
      double freq = getFrequency();
      if (isConnected() && freq > expectedMinFrequency && freq < expectedMaxFrequency) {
        return true;
      }

      Timer.delay(0.05);
    }

    return false;
  }
}
