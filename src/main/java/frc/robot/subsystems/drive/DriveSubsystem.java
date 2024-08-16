package frc.robot.subsystems.drive;

import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {

  private final SwerveIO io;
  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  public DriveSubsystem(SwerveIO io) {
    this.io = io;
    org.littletonrobotics.junction.Logger.recordOutput("Test Measurement", true);
  }

  public void drive(double Xmps, double Ymps, double OmegaRadps) {
    io.drive(Xmps, Ymps, OmegaRadps);
  }

  public void resetGyro() {
    io.resetGyro();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("Gyro Rotation", () -> inputs.gyroRotation));
  }
}
