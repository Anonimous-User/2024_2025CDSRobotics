package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.RevUtil.checkRevError;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.util.PIDGains;
import org.littletonrobotics.junction.Logger;

public class ArmSimple extends SubsystemBase {
  private static final LoggedTunableNumber tunnableGearRatio =
      new LoggedTunableNumber("Arm/kGearRatio", ArmConstants.kArmGearRatio);
  private static double positionFactor = ArmConstants.kPositionFactor;
  private static double velocityFactor = ArmConstants.kVelocityFactor;

  private CANSparkMax leaderMotor;
  private CANSparkMax followMotor;
  public RelativeEncoder leaderEncoder;
  private RelativeEncoder followerEncoder;

  private SparkPIDController leadController;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;

  public ArmSimple() {
    followMotor = new CANSparkMax(ArmConstants.kArmFollowerCanId, CANSparkMax.MotorType.kBrushless);
    leaderMotor = new CANSparkMax(ArmConstants.kArmLeaderCanId, CANSparkMax.MotorType.kBrushless);
    followerEncoder = followMotor.getEncoder();
    leaderEncoder = leaderMotor.getEncoder();

    // Config Hardware
    checkRevError(followMotor.restoreFactoryDefaults());
    checkRevError(leaderMotor.restoreFactoryDefaults());

    checkRevError(followMotor.follow(leaderMotor, true)); // + is rotating up from front floor

    leaderEncoder.setPositionConversionFactor(1.0);
    leaderEncoder.setVelocityConversionFactor(1.0);

    // Limits
    CANSparkMax motors[] = {leaderMotor, followMotor};
    for (CANSparkMax motor : motors) {
      checkRevError(motor.setSmartCurrentLimit(ArmConstants.kCurrentLimit));
      checkRevError(motor.enableVoltageCompensation(kVoltageCompenstation));
      checkRevError(motor.setIdleMode(CANSparkBase.IdleMode.kBrake));
      // checkRevError(motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true));
      // checkRevError(motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true));
      // checkRevError(
      //     motor.setSoftLimit(
      //         CANSparkBase.SoftLimitDirection.kForward, (float) ArmConstants.kSoftLimitForward));
      // checkRevError(
      //     motor.setSoftLimit(
      //         CANSparkBase.SoftLimitDirection.kReverse, (float) ArmConstants.kSoftLimitReverse));
    }

    leadController = leaderMotor.getPIDController();
    PIDGains.setSparkMaxGains(leadController, ArmConstants.kArmPositionGains);

    checkRevError(followMotor.burnFlash());
    checkRevError(leaderMotor.burnFlash());

    m_setpoint = ArmConstants.kHomePosition;

    m_timer = new Timer();
    m_timer.start();

    updateMotionProfile();
  }

  /**
   * Sets the target position and updates the motion profile if the target position changed.
   *
   * @param _setpoint The new target position in radians.
   */
  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  /**
   * Update the motion profile variables based on the current setpoint and the pre-configured motion
   * constraints.
   */
  private void updateMotionProfile() {
    m_startState =
        new TrapezoidProfile.State(leaderEncoder.getPosition(), leaderEncoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(ArmConstants.kArmMotionConstraint);
    m_timer.reset();
  }

  /**
   * Drives the arm to a position using a trapezoidal motion profile. This function is usually
   * wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
   *
   * <p>This function updates the motor position control loop using a setpoint from the trapezoidal
   * motion profile. The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    m_feedforward = -0.5;
    leadController.setReference(
        m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  public Command armSpeaker(ArmSimple arm) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private double pos;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            pos = -2.5;
            setTargetPosition(pos);
            runAutomatic();
          }

          @Override
          public void execute() {}

          @Override
          public boolean isFinished() {
            return m_timer.get() > 3;
          }

          @Override
          public void end(boolean interrupted) {
            setTargetPosition(leaderEncoder.getPosition());
            runAutomatic();
          }
        };

    newCommand.addRequirements(this, arm);

    return newCommand;
  }

  public Command armAmp(ArmSimple arm) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private double pos;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            pos = -24.5;
            setTargetPosition(pos);
            runAutomatic();
          }

          @Override
          public void execute() {}

          @Override
          public boolean isFinished() {
            return m_timer.get() > 3 || MathUtil.isNear(pos, leaderEncoder.getPosition(), 0.1);
          }

          @Override
          public void end(boolean interrupted) {
            setTargetPosition(leaderEncoder.getPosition());
            runAutomatic();
          }
        };

    newCommand.addRequirements(this, arm);

    return newCommand;
  }

  public Command armGround(ArmSimple arm) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private double pos;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            pos = 0;
            setTargetPosition(pos);
            runAutomatic();
          }

          @Override
          public void execute() {}

          @Override
          public boolean isFinished() {
            return m_timer.get() > 3 || MathUtil.isNear(pos, leaderEncoder.getPosition(), 0.2);
          }

          @Override
          public void end(boolean interrupted) {
            setTargetPosition(leaderEncoder.getPosition());
            runAutomatic();
          }
        };

    newCommand.addRequirements(this, arm);

    return newCommand;
  }

  public Command armChilling(ArmSimple arm) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private double pos;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            pos = -17;
            setTargetPosition(pos);
            runAutomatic();
          }

          @Override
          public void execute() {}

          @Override
          public boolean isFinished() {
            return m_timer.get() > 3 || MathUtil.isNear(pos, leaderEncoder.getPosition(), 0.1);
          }

          @Override
          public void end(boolean interrupted) {
            setTargetPosition(leaderEncoder.getPosition());
            runAutomatic();
          }
        };

    newCommand.addRequirements(this, arm);

    return newCommand;
  }

  public Command armVert(ArmSimple arm) {
    Command newCommand =
        new Command() {
          private Timer m_timer;
          private double pos;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            pos = -21;
            setTargetPosition(pos);
            runAutomatic();
          }

          @Override
          public void execute() {}

          @Override
          public boolean isFinished() {
            return m_timer.get() > 3 || MathUtil.isNear(pos, leaderEncoder.getPosition(), 0.1);
          }

          @Override
          public void end(boolean interrupted) {
            setTargetPosition(leaderEncoder.getPosition());
            runAutomatic();
          }
        };

    newCommand.addRequirements(this, arm);

    return newCommand;
  }

  // /**
  //  * Drives the arm using the provided power value (usually from a joystick). This also adds in
  // the
  //  * feedforward value which can help counteract gravity.
  //  *
  //  * @param _power The motor power to apply.
  //  */
  // public void runManual(double _power) {
  //   SmartDashboard.putBoolean("hi", true);
  //   // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly
  // and
  //   // passively
  //   m_setpoint = leaderEncoder.getPosition();
  //   updateMotionProfile();
  //   // update the feedforward variable with the newly zero target velocity
  //   m_feedforward = 0.0;
  //   //        ArmConstants.kArmFeedforward.calculate(
  //   //            leaderEncoder.getPosition() + ArmConstants.kArmZeroCosineOffset,
  //   //            m_targetState.velocity);
  //   // set the power of the motor
  //   leaderMotor.set(_power + (m_feedforward / 12.0));
  //   SmartDashboard.putBoolean("hi2", true);
  //   m_manualValue = _power; // this variable is only used for logging or debugging if needed
  // }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    Logger.recordOutput("Arm/Encoder/Position", leaderEncoder.getPosition());
    Logger.recordOutput("Arm/Velocity", leaderEncoder.getVelocity());
    Logger.recordOutput("Arm/AppliedOutput", leaderMotor.getAppliedOutput());
    Logger.recordOutput("Arm/OutputCurrent", leaderMotor.getOutputCurrent());
  }
}
