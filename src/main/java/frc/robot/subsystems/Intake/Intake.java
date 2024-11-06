package frc.robot.subsystems.Intake;

import frc.robot.constants.IntakeConstants;
import frc.robot.standards.OpenLoopSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class Intake extends MeasurableSubsystem implements OpenLoopSubsystem {

  // Private Variables
  private float totalBreaks = 0;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeState curState = IntakeState.IDLE;

  // Constructor
  public Intake(IntakeIO io) {
    this.io = io;
  }

  // Getter/Setter Methods
  public IntakeState getState() {
    return curState;
  }

  // Periodic Function
  @Override
  public void periodic() {
    // Read Inputs
    io.updateInputs(inputs);
    Logger.processInputs("IntakeInputs", inputs);

    // State Machine
    switch (curState) {
      case Running:
        // wait for break 3 times
        if (inputs.revBeamOpen == false) {
          totalBreaks += 1;
        } else {
          totalBreaks = 0;
        }

        if (totalBreaks >= inputs.requiredBreaks) {
          curState = IntakeState.PickingUp;
        }
        break;
      case PickingUp:
        break;
      case NotePassed:
        break;
      default:
        break;
    }

    // Log Outputs
    Logger.recordOutput("Intake/curState", curState);
    Logger.recordOutput("Intake/totalBreaks", totalBreaks);
  }

  // Grapher
  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
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

  public void ReverseMotors() {
    curState = IntakeState.NotePassed;
    io.SetSpeed(IntakeConstants.kReversingSpeed);
  }

  public void StartIntake() {
    curState = IntakeState.Running;
    io.SetSpeed(IntakeConstants.kIntakingSpeed);
  }
}
