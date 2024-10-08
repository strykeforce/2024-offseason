package frc.robot.subsystems.exiter.ExiterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.exiter.ExiterSubsystem;

public class SameSpeed extends Command {

  private ExiterSubsystem exitersubsystem;
  private double bothSpeed;

  public SameSpeed(ExiterSubsystem exitersubsystem, double bothSpeed) {
    this.exitersubsystem = exitersubsystem;
    this.bothSpeed = bothSpeed;
  }

  @Override
  public void initialize() {
    exitersubsystem.setSpeed(bothSpeed, bothSpeed);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return exitersubsystem.atSpeed();
  }
}
