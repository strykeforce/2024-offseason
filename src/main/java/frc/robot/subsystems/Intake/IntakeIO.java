package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double velocity = 0.0;
    // open = beam isnt broken
    public boolean revBeamOpen = false;
    public int requiredBreaks = 3;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void SetSpeed(double speed) {}

}
