// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.EndIntakeCommand;
import frc.robot.commands.intake.StartIntakeCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOFX;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  private final XboxController xboxController = new XboxController(0);
  private final Intake intake;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  public RobotContainer() {
    intake = new Intake(new IntakeIOFX());
    intake.registerWith(telemetryService);
    telemetryService.start();
    configureBindings();
  }

  private void configureBindings() {

    new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(new StartIntakeCommand(intake));
    new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(new EndIntakeCommand(intake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
