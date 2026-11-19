package first.robot;

import first.robot.subsystems.swervedrive.SwerveDriveSubsystem;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import yams.commands2.swerve.SwerveInputStream;

public class RobotContainer
{

  final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SwerveDriveSubsystem swerve = new SwerveDriveSubsystem();

  // Xbox sticks read negative when pushed forward/left, so every axis is negated:
  // left stick forward drives away from the driver station, right stick right spins clockwise.
  private final SwerveInputStream driveAngularVelocity =
      swerve.getAngularVelocityStream(
                () -> -driverXbox.getLeftY(),
                () -> -driverXbox.getLeftX(),
                () -> -driverXbox.getRightX())
            .withDeadband(0.05)
            .withAllianceRelativeControl();

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings()
  {
    // Default drive command
    swerve.setDefaultCommand(swerve.driveFieldRelative(driveAngularVelocity));

    // Utility mode is for live tuning: hold the drive idle so the default command doesn't fight it.
    RobotModeTriggers.utility().whileTrue(Commands.idle(swerve).withName("Live Tuning"));

    // Zero the gyro with Menu + View. Use this if the field-relative heading drifts.
    driverXbox.menu().and(driverXbox.view()).onTrue(swerve.zeroGyro());
  }
  Command getAutonomousCommand()
  {
    return Commands.idle();
  }
}