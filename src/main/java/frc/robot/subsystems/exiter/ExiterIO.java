package frc.robot.subsystems.exiter;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ExiterIO {

  @AutoLog
  public static class ExiterIOInputs {
    public double position = 0.0;
    public double velocityLeft = 0.0;
    public double velocityRight = 0.0;
  }

  public default void updateInputs(ExiterIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void setSpeed(double leftSpeed, double rightSpeed) {}
}
