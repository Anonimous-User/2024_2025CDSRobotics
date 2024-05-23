package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.util.AllianceFlipUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Trajectory;

public class InitializePose extends Command {
  private Drive drive;
  private int position;
  private boolean initialized;

  public InitializePose(Drive drive, int position) {
    this.drive = drive;
    this.position = position;
    this.initialized = false;
  }

  @Override
  public void execute() {
    // Need to wait for the DriverStation alliance to be present for the AllianceFlipUtil
    // to get the correct side of the field.
    if (DriverStation.getAlliance().isPresent()) {
      switch (position) {
        case 1:
          {
            drive.setPose(AllianceFlipUtil.apply(Trajectory.startingBackOnAmpFace));
            break;
          }
        case 2:
          {
            drive.setPose(AllianceFlipUtil.apply(Trajectory.startingBackOnCenterFace));
            break;
          }
        case 3:
          {
            drive.setPose(AllianceFlipUtil.apply(Trajectory.startingBackOnSourceFace));
            break;
          }
      }
      initialized = true;
    }
  }

  @Override
  public boolean isFinished() {
    return initialized;
  }
}
