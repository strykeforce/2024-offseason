// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.drivePathHandler;
import frc.robot.commands.drive.driveTeleop;
import frc.robot.commands.drive.resetGyro;
import frc.robot.commands.drive.rotatingRobot;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.PathHandler.PathHandler;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import java.util.List;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  private final DriveSubsystem driveSubsystem;
  private final PathHandler pathHandler;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flyskyJoystick = new FlyskyJoystick(driveJoystick);

  private Swerve swerve;

  public RobotContainer() {
    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);
    pathHandler = new PathHandler(driveSubsystem);

    configureDriverBindings();
    configureTelemtry();
  }

  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        new driveTeleop(
            () -> flyskyJoystick.getFwd(),
            () -> flyskyJoystick.getStr(),
            () -> flyskyJoystick.getYaw(),
            driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new resetGyro(driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_SWE.id).onTrue(new rotatingRobot(driveSubsystem));
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(new drivePathHandler(driveSubsystem, pathHandler, "standardAutonStart", List.of(1, 2, 3,2,1,3,2), 3));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void configureTelemtry() {
    driveSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }
}
