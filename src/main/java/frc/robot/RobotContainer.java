// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.driveTeleop;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem;
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flyskyJoystick = new FlyskyJoystick(driveJoystick);

  private Swerve swerve;

  public RobotContainer() {
    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);
    configureDriverBindings();
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new driveTeleop(
            () -> flyskyJoystick.getFwd(),
            () -> flyskyJoystick.getStr(),
            () -> flyskyJoystick.getYaw(),
            driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
