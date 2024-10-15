// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.driveTeleop;
import frc.robot.commands.drive.resetGyro;
import frc.robot.commands.drive.rotatingRobot;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.exiter.*;
import frc.robot.subsystems.exiter.ExiterCommands.SameSpeed;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem;
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flyskyJoystick = new FlyskyJoystick(driveJoystick);

  private Swerve swerve;

  private final ExiterSubsystem exiter;
  private XboxController xboxController;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  public RobotContainer() {
    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);
    exiter = new ExiterSubsystem(new ExiterIOFX());
    exiter.registerWith(telemetryService);
    telemetryService.start();
    xboxController = new XboxController(1);
    configureDriverBindings();
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new driveTeleop(
            () -> flyskyJoystick.getFwd(),
            () -> flyskyJoystick.getStr(),
            () -> flyskyJoystick.getYaw(),
            driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new resetGyro(driveSubsystem));
    new JoystickButton(driveJoystick, Button.SWD.id).onTrue(new rotatingRobot(driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
