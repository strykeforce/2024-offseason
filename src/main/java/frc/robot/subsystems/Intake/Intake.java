package frc.robot.subsystems.Intake;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;


import frc.robot.standards.OpenLoopSubsystem;

public class Intake extends MeasurableSubsystem implements OpenLoopSubsystem {

    private float totalBreaks;
    private boolean fwdBeamOpen;

    private IntakeIOFX Motor;
    private double speed;

    // Private Variables
  private IntakeState curState = IntakeState.Running;

  // Getter/Setter Methods
  public IntakeState getState() {
    return curState;
  }

  // Periodic Function
  @Override
  public void periodic() {

    // State Machine
    switch (curState) {
      case Running:
        Motor.SetSpeed(0);
        //wait for break 3 times
        if(fwdBeamOpen = true)
        {
          totalBreaks += 1;
        }else{
          totalBreaks = 0;
        }

        if(totalBreaks >= 3)
        {
          curState = IntakeState.PickingUp;
        }
        break;
      case PickingUp:
        Motor.SetSpeed(speed);
        break;
      case NotePassed:
        Motor.SetSpeed(-speed);
        break;
      default:
        break;
    }

    // Log Outputs
    Logger.recordOutput("Example/curState", curState.ordinal());
  }

  // Grapher
  @Override
  public void registerWith(TelemetryService telemetryService) {

    super.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> curState.ordinal()));
  }

  // State Enum
  public enum IntakeState {
    IDLE,
    NotePassed, 
    Running, 
    PickingUp
  }
  
    public void ReverseMotors()
    {
      curState = IntakeState.NotePassed;
    }

    public void Fired()
    {
      curState = IntakeState.Running;
    }
}
