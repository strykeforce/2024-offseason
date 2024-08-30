package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs {
    public double odometryX = 0.0;
    public double odometryY = 0.0;
    public double odometryCalculated = 0.0;
    public double gyroRotation = 0.0;
    public Rotation2d gyroRotation2d = new Rotation2d();
    public double odometryRotation2D = 0.0;
    public Pose2d poseMeters = new Pose2d();
  }

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void drive(double Xmps, double Ymps, double OmegaRadps) {}

  public default void move(double Xmps, double Ymps, double OmegaRadps) {}

  public default void resetGyro() {}

  public default void resetOdometry(Pose2d pose2d) {}
}
