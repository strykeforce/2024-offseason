//if you see this, tell me what else i need to add in the notes file


package frc.robot.subsystems.Magazine;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;

import frc.robot.standards.ClosedLoopPosSubsystem;
import frc.robot.subsystems.Magazine.MagazineIOFX;

public class MagazineSubsystem extends MeasurableSubsystem implements ClosedLoopPosSubsystem {
  // Private Variables
  private final MagazineIO io;
  private final ExampleIOInputsAutoLogged inputs = new ExampleIOInputsAutoLogged();
  private double setpoint = 0.0;

  // Constructor
  public MagazineSubsystem(MagazineIO io) {
    this.io = io;
    zero();
  }
  //change if needed
    float speed = 1;

    public void intake(){
        new MagazineIOFX().spin(speed);
    }
    public void stopIntake(){
        new MagazineIOFX().stopSpinning();
    }
}