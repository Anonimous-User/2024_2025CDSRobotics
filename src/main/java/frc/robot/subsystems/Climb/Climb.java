package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final DoubleSolenoid reach = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 15, 14);
  private Value state = Value.kReverse;

  public Climb() {
    state = Value.kReverse;
    reach.set(state);
  }

  public void use() {
    if (state == Value.kReverse) {
      state = Value.kForward;
    } else {
      state = Value.kReverse;
    }
    reach.set(state);
  }
}
