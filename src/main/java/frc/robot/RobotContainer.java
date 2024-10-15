// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.exiter.*;
import frc.robot.subsystems.exiter.ExiterCommands.SameSpeed;

public class RobotContainer {
  private final ExiterSubsystem exiter;
  private XboxController xboxController;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  public RobotContainer() {
    exiter = new ExiterSubsystem(new ExiterIOFX());
    exiter.registerWith(telemetryService);
    telemetryService.start();
    xboxController = new XboxController(1);
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new SameSpeed(exiter, 40));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
