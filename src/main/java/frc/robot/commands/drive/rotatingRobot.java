package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class rotatingRobot extends Command {
  private final DriveSubsystem driveSubsystem;

  public rotatingRobot(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  public void initialize() {
    driveSubsystem.rotateRobot();
  }
}
