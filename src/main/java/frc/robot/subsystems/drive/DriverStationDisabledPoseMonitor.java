package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.OptionalInt;

// This watches the DriverStation alliance/location only while the station is disabled.
// If the position changes, the drive pose is updated. If the position does not change,
// the pose is not adjusted which means if a LimeLight updates the pose it will
// be the most recent unless the DriverStation changes in which case it would be the
// most up-to-date pose until a LimeLight happens to update.
public class DriverStationDisabledPoseMonitor extends CommandBase {
  private Drive drive;
  private OptionalInt previousLocation = DriverStation.getLocation();
  private Optional<Alliance> previousAlliance = DriverStation.getAlliance();
  public boolean initialized = false;

  public DriverStationDisabledPoseMonitor(Drive drive) {
    this.drive = drive;
  }

  public void execute() {
    // Only adjust the Robot pose while the driver station is disabled
    if (DriverStation.isEnabled()) {
      OptionalInt location = DriverStation.getLocation();
      Optional<Alliance> alliance = DriverStation.getAlliance();

      // Only adjust the Robot pose if we have an Alliance and Location available
      if (alliance.isPresent() && location.isPresent()) {

        // Only adjust if we have not already adjusted or the Alliance/Location changes
        if (!initialized || previousLocation != location || previousAlliance != alliance) {
          switch (location.getAsInt()) {
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
          previousAlliance = alliance;
          previousLocation = location;
          initialized = true;
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return initialized;
  }
}
