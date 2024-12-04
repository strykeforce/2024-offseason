// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.drivePathHandler;
import frc.robot.commands.drive.driveTeleop;
import frc.robot.commands.drive.resetGyro;
import frc.robot.commands.drive.rotatingRobot;
import frc.robot.commands.intake.EndIntakeCommand;
import frc.robot.commands.intake.StartIntakeCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOFX;
import frc.robot.subsystems.PathHandler.PathHandler;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.exiter.*;
import frc.robot.subsystems.exiter.ExiterCommands.SameSpeed;
import java.util.List;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  private final PathHandler pathHandler;
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  private Swerve swerve;
  private final DriveSubsystem driveSubsystem;
  private final ExiterSubsystem exiter;
  private final Intake intake;

  private XboxController xboxController;
  private final Joystick driveJoystick = new Joystick(0);
  private final FlyskyJoystick flyskyJoystick = new FlyskyJoystick(driveJoystick);

  public RobotContainer() {
    swerve = new Swerve();
    driveSubsystem = new DriveSubsystem(swerve);
    exiter = new ExiterSubsystem(new ExiterIOFX());
    intake = new Intake(new IntakeIOFX());
    pathHandler = new PathHandler(driveSubsystem, intake);

    xboxController = new XboxController(1);
    configureTelemetry();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureTelemetry() {
    exiter.registerWith(telemetryService);
    intake.registerWith(telemetryService);
    driveSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }
  private void configureDriverBindings() {
    System.out.println("Git this is not up to date");
    driveSubsystem.setDefaultCommand(
        new driveTeleop(
            () -> flyskyJoystick.getFwd(),
            () -> flyskyJoystick.getStr(),
            () -> flyskyJoystick.getYaw(),
            driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_SWC.id).onTrue(new resetGyro(driveSubsystem));
    new JoystickButton(driveJoystick, Button.M_SWE.id).onTrue(new rotatingRobot(driveSubsystem));
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(
            new drivePathHandler(
                driveSubsystem,
                pathHandler,
                intake,
                "standardAutonStart",
                List.of(1, 2, 3, 2, 1, 2,3,2,1,2,3,2,1,2,3),
                6));
  }

  private void configureOperatorBindings() {
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new SameSpeed(exiter, 40));

    new JoystickButton(xboxController, XboxController.Button.kA.value)
        .onTrue(new StartIntakeCommand(intake));
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new EndIntakeCommand(intake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void configureTelemtry() {
    driveSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }
}
