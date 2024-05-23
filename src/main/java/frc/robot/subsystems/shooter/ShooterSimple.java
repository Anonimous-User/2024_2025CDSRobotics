package frc.robot.subsystems.shooter;

import static frc.robot.util.RevUtil.checkRevError;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.arm.ArmSimple;

public class ShooterSimple extends SubsystemBase {

  private static final LoggedTunableNumber bottomPower =
      new LoggedTunableNumber("Shooter/BottomPower", ShooterConstants.kBottomPower);
  private static final LoggedTunableNumber topPower =
      new LoggedTunableNumber("Shooter/TopPower", ShooterConstants.kTopPower);
  private CANSparkMax m_topMotor;
  private CANSparkMax m_bottomMotor;
  private ArmSimple arm;

  private boolean m_launcherRunning;

  /** Creates a new LauncherSubsystem. */
  public ShooterSimple(ArmSimple a) {
    arm = a;
    m_topMotor = new CANSparkMax(ShooterConstants.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor =
        new CANSparkMax(ShooterConstants.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);

    checkRevError(m_topMotor.restoreFactoryDefaults());
    checkRevError(m_bottomMotor.restoreFactoryDefaults());

    m_topMotor.setInverted(false);
    checkRevError(m_topMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit));
    checkRevError(m_topMotor.setIdleMode(IdleMode.kBrake));

    m_bottomMotor.setInverted(false);
    checkRevError(m_bottomMotor.setSmartCurrentLimit(ShooterConstants.kCurrentLimit));
    checkRevError(m_bottomMotor.setIdleMode(IdleMode.kBrake));

    checkRevError(m_topMotor.burnFlash());
    checkRevError(m_bottomMotor.burnFlash());

    m_launcherRunning = false;
  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runShooter() {
    m_launcherRunning = true;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    SmartDashboard.putBoolean("Shooter/Running", m_launcherRunning);
    SmartDashboard.putNumber("Shooter/TopAppliedOutput", m_topMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/BottomAppliedOutput", m_bottomMotor.getAppliedOutput());

    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      if (DriverStation.isAutonomousEnabled()) {
        m_topMotor.set(0.4);
        m_bottomMotor.set(0.38);
      } else {
        m_topMotor.set(0.45);
        m_bottomMotor.set(0.43);
      }
    } else {
      m_topMotor.set(0.0);
      m_bottomMotor.set(0.0);
    }
  }
}
