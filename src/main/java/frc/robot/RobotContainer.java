// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drive.Module.ModulePosition.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team1701.controls.RumbleController;
import frc.lib.team6328.util.SparkMaxBurnManager;
import frc.robot.commands.*;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.Climb.*;
import frc.robot.subsystems.arm.ArmSimple;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeSimple;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.sensors.NoteSensorIODIO;
import frc.robot.subsystems.shooter.ShooterSimple;
import frc.robot.subsystems.vision.LimelightWithBotPose;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive = null;
  private ArmSimple armSimple = null;
  private IntakeSimple intake = null;
  private ShooterSimple shooter = null;
  private Climb climb = null;

  private Candle candle = null;

  private NoteSensor sensor = null;
  HttpCamera limelight =
      new HttpCamera(
          "limelight", "http://10.49.51.60:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
  MjpegServer llcam;
  UsbCamera cam;

  LimelightWithBotPose limelightWithBotPose = new LimelightWithBotPose("limelight");

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController controller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Pose2d ampPose;

  // Create the constraints to use while pathfinding
  PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  private Command pathtoampCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    llcam = CameraServer.startAutomaticCapture(limelight);

    if (USE_USB_CAMERA_ON_RIO2) {
      cam = CameraServer.startAutomaticCapture(0);
      cam.setResolution(200, 100);
    }

    drive =
        new Drive(
            // new GyroIOPigeon1(DriveConstants.PIGEON1_CAN_ID),
            new GyroIOPigeon2(DriveConstants.PIGEON2_CAN_ID, DriveConstants.PIGEON2_CANBUS),
            new ModuleIOSparkMax(DriveConstants.moduleConfigs.get(FRONT_LEFT)),
            new ModuleIOSparkMax(DriveConstants.moduleConfigs.get(FRONT_RIGHT)),
            new ModuleIOSparkMax(DriveConstants.moduleConfigs.get(REAR_LEFT)),
            new ModuleIOSparkMax(DriveConstants.moduleConfigs.get(REAR_RIGHT)));
    intake = new IntakeSimple();
    armSimple = new ArmSimple();
    shooter = new ShooterSimple(armSimple);
    sensor = new NoteSensor(new NoteSensorIODIO());
    climb = new Climb();
    candle = new Candle(62);

    // set the intake to stop (0 power) when no other command is running
    intake.setDefaultCommand(new RunCommand(() -> intake.setPower(0.0), intake));

    // configure the launcher to stop when no other command is running
    shooter.setDefaultCommand(new RunCommand(() -> shooter.stopLauncher(), shooter));

    NamedCommands.registerCommand("initial1", new InitializePose(drive, 1));
    NamedCommands.registerCommand("initial2", new InitializePose(drive, 2));
    NamedCommands.registerCommand("initial3", new InitializePose(drive, 3));
    NamedCommands.registerCommand(
        "downandshoot",
        new SequentialCommandGroup(
            armSimple.armGround(armSimple), new ShootSpeaker(shooter, intake)));
    NamedCommands.registerCommand("armdown", armSimple.armGround(armSimple));
    NamedCommands.registerCommand("shoot", new ShootSpeaker(shooter, intake));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("drive forward", new DriveForward(drive));

    autoChooser.addDefaultOption(
        "Shoot",
        new SequentialCommandGroup(
            armSimple.armGround(armSimple), new ShootSpeaker(shooter, intake)));

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // if (RUNNING_FULL_FEATURED_ROBOT) {
    //   autoChooser.addOption(
    //       "drive forward",
    //       new SequentialCommandGroup(armSimple.armGround(armSimple), new DriveForward(drive)));
    //   autoChooser.addOption(
    //       "shoot and drive",
    //       new SequentialCommandGroup(
    //           armSimple.armGround(armSimple),
    //           new ShootSpeaker(shooter, intake),
    //           new DriveToSide(drive)));
    //   autoChooser.addOption(
    //       "shoot",
    //       new SequentialCommandGroup(
    //           armSimple.armGround(armSimple), new ShootSpeaker(shooter, intake)));
    // }

    if (USE_PATH_PLANNING_STUFF) {
      // Since we are using a holonomic drivetrain, the rotation component of this pose
      // represents the goal holonomic rotation
      // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      ampPose = new Pose2d(1.91, 7.67, Rotation2d.fromDegrees(90));
      //    } else {
      //      ampPose = new Pose2d(14.65, 7.67, Rotation2d.fromDegrees(90));
      //    }

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      pathtoampCommand =
          AutoBuilder.pathfindToPose(
              ampPose,
              constraints,
              0.0, // Goal end velocity in meters/sec
              0.0 // Rotation delay distance in meters. This is how far the robot should travel
              // before
              // attempting to rotate.
              );
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger robotRelativeBooleanSupplier = driver.leftBumper();
    Trigger robotSlowBooleanSupplier = driver.rightBumper();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            robotRelativeBooleanSupplier,
            robotSlowBooleanSupplier));
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    //    driver
    //        .b()
    //        .onTrue(
    //            Commands.runOnce(
    //                    () ->
    //                        drive.setPose(
    //                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                    drive)
    //                .ignoringDisable(true));
    // driver.a().onTrue(new TurnAngleCommand(drive, Rotation2d.fromDegrees(10)));
    driver.a().onTrue(new InstantCommand(() -> climb.use()));

    if (USE_PATH_PLANNING_STUFF) {
      // slign with amp using limelight
      driver.y().whileTrue(new AlignWithAmp(drive, limelightWithBotPose));

      // driver.b().onTrue(pathtoampCommand);
      //      driver.b().onTrue(new PathfindThenFollowPathHolonomic());
    }

    var rumbleDriver = new RumbleController(driver.getHID());
    var rumbleController = new RumbleController(controller.getHID());

    controller
        .rightBumper()
        .whileTrue(
            intake
                .intake(0.4)
                .until(sensor::noteSensedByLeftSensor)
                .andThen(
                    Commands.parallel(
                        intake.retract(), // We have a Note, retract and notify
                        rumbleDriver
                            .rumblePulses(2)
                            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                        rumbleController
                            .rumblePulses(2)
                            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                        new CandleNoteDetected(candle, sensor))));

    // shooter controls (button to pre-spin the launcher and button to launch)

    // controller.leftBumper().whileTrue(new RunCommand(() -> shooter.runShooter(), shooter));

    // controller.a().onTrue(intake.feedLauncher(shooter));

    // controller.x().whileTrue(intake.intake(-0.5));

    controller.leftBumper().onTrue(new ShootAmp(shooter, intake));
    controller.leftStick().whileTrue(new RunCommand(() -> shooter.runShooter()));
    controller
        .leftStick()
        .and(controller.leftBumper())
        .whileTrue(new InstantCommand(() -> intake.setPower(1)));
    controller.rightStick().whileTrue(new Outtake(intake));

    if (RUNNING_FULL_FEATURED_ROBOT) {
      controller.a().onTrue(armSimple.armAmp(armSimple));
      controller.x().onTrue(armSimple.armChilling(armSimple));
      controller.y().onTrue(armSimple.armGround(armSimple));
      controller.b().onTrue(armSimple.armSpeaker(armSimple));
    }

    // Endgame alert triggers
    double endGameAlertInSeconds = 10.0;
    var teleopEnabled = new Trigger(DriverStation::isTeleopEnabled);
    teleopEnabled
        .and(
            () ->
                DriverStation.isFMSAttached()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= endGameAlertInSeconds)
        .onTrue(
            Commands.parallel(
                rumbleDriver
                    .rumblePulses(3)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                rumbleController
                    .rumblePulses(3)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        armSimple.armGround(armSimple), new ShootSpeaker(shooter, intake), autoChooser.get());
  }
}
