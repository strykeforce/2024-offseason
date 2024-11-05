package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake.Intake;

public class EndIntakeCommand extends InstantCommand {

  private Intake intake;

  public EndIntakeCommand(Intake IntakeCon) {
    addRequirements(IntakeCon);
    intake = IntakeCon;
  }

  @Override
  public void initialize() {
    intake.ReverseMotors();
  }
}
