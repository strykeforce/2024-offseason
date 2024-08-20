package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.DriveConstants;
import java.util.Set;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {

  private final SwerveIO io;
  private final ProfiledPIDController omegaShootTrackController;
  private final ProfiledPIDController omegaSpinController;
  private final ProfiledPIDController omegaController;
  private final PIDController xController;
  private final PIDController yController;
  private final HolonomicDriveController holonomicController;
  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private State holoInput = new State();
  private Rotation2d holoAngle = new Rotation2d();
  private ChassisSpeeds holoOutput = new ChassisSpeeds();

  public DriveSubsystem(SwerveIO io) {
    this.io = io;
    org.littletonrobotics.junction.Logger.recordOutput("Test Measurement", true);

    omegaShootTrackController =
        new ProfiledPIDController(
            DriveConstants.kPOmegaSpin,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmegaSpin));
    omegaShootTrackController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    omegaSpinController =
        new ProfiledPIDController(
            DriveConstants.kPOmegaSpin,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmegaSpin));
    omegaSpinController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
    omegaController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmegaPath));
    omegaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    xController =
        new PIDController(
            DriveConstants.kXPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
    yController =
        new PIDController(
            DriveConstants.kYPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);

    holonomicController = new HolonomicDriveController(xController, yController, omegaController);
    holonomicController.setEnabled(true);
  }

  public void drive(double Xmps, double Ymps, double OmegaRadps) {
    io.drive(Xmps, Ymps, OmegaRadps);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve X", Xmps);
    org.littletonrobotics.junction.Logger.recordOutput("Swerve Y", Ymps);
  }

  public void move(double Xmps, double Ymps, double OmegaRadps) {
    io.move(Xmps, Ymps, OmegaRadps);
  }

  public void holonomicCalculator(State wantedState, Rotation2d wantedAngle) {
    holoInput = wantedState;
    holoAngle = wantedAngle;
    holoOutput = holonomicController.calculate(inputs.poseMeters, wantedState, wantedAngle);
    io.move(
        holoOutput.vxMetersPerSecond,
        holoOutput.vyMetersPerSecond,
        holoOutput.omegaRadiansPerSecond);
  }

  public void driveAutonXController(State wantedState, Rotation2d wantedAngle, double driveY) {
    holoInput = wantedState;
    holoAngle = wantedAngle;
    holoOutput = holonomicController.calculate(inputs.poseMeters, wantedState, wantedAngle);
    io.move(holoOutput.vxMetersPerSecond, driveY, holoOutput.omegaRadiansPerSecond);
  }

  public void resetGyro() {
    io.resetGyro();
  }

  public void rotateRobot() {
    io.move(0,0, 0.33);
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
