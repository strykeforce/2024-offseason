package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class driveTeleop extends Command {
  private DoubleSupplier fwdStick;
  private DoubleSupplier strStick;
  private DoubleSupplier yawStick;
  private final DriveSubsystem driveSubsystem;
  private double[] rawValues = new double[3];

  public driveTeleop(
      DoubleSupplier fwdStick,
      DoubleSupplier strStick,
      DoubleSupplier yawStick,
      DriveSubsystem driveSubsystem) {
    this.fwdStick = fwdStick;
    this.strStick = strStick;
    this.yawStick = yawStick;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    rawValues[0] = fwdStick.getAsDouble();
    rawValues[1] = strStick.getAsDouble();
    rawValues[2] = yawStick.getAsDouble();
    driveSubsystem.drive(-fwdStick.getAsDouble(), -strStick.getAsDouble(), -yawStick.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0);
  }
}
