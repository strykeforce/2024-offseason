package frc.robot.subsystems.Magazine;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

/**
 * Interface for controlling the Magazine subsystem.
 */
public interface MagazineIO {
      @AutoLog
  public static class IOInputs {
    public boolean beambreak = false;
    public double velocity = 0.0;
  }

  public default void updateInputs(IOInputs inputs) {}

    boolean spin(double speed);
    
    void stopSpinning();

    double getSpeed();

    boolean atSpeed();

    double isBeamBroken();

    public default void registerWith(TelemetryService telemetryService) {}
}
