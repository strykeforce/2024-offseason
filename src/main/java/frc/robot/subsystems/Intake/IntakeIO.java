package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface IntakeIO {

  @AutoLog
  public static class ExampleIOInputs {
    public double velocity = 0.0;
    //open = beam isnt broken
    public boolean fwdBeamOpen = false;
  }

  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void zero() {}

  public default void registerWith(TelemetryService telemetryService) {}
}
