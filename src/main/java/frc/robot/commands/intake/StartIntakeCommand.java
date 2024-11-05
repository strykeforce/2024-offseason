package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.Intake;

public class StartIntakeCommand extends InstantCommand {

  private Intake intake;

  public StartIntakeCommand(Intake IntakeCon) {
    addRequirements(IntakeCon);
    intake = IntakeCon;
  }

  @Override
  public void initialize() {
    intake.StartIntake();
  }
}
