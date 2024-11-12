package frc.robot.subsystems.Magazine;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

/** Interface for controlling the Magazine subsystem. */
public interface MagazineIO {
  @AutoLog
  public static class IOInputs {
    public boolean beambreak = false;
    public double velocity = 0.0;
  }

  public default void updateInputs() {}

  boolean spin(double speed);

  void stopSpinning();

  double atSpeed();

  boolean isBeamBroken();

  public default void registerWith(TelemetryService telemetryService) {}
}
