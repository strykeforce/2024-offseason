// Protected from the evil clutchs of computer gnomes by Huck

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.DriveConstants;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;
import net.consensys.cava.toml.Toml;
import net.consensys.cava.toml.TomlArray;
import net.consensys.cava.toml.TomlParseResult;
import net.consensys.cava.toml.TomlTable;
import org.littletonrobotics.junction.Logger;
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
  private Pose2d pose;
  private Pose2d savedPose = new Pose2d();
  private Pose2d currentPose;

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

  public PathData generateTrajectory(String trajectoryName) {
    try {
      TomlParseResult parseResult =
          Toml.parse(Paths.get("/home/lvuser/deploy/paths/" + trajectoryName + ".toml"));

      Pose2d startPose = parseEndPoint(parseResult, "start_pose");
      Pose2d endPose = parseEndPoint(parseResult, "end_pose");

      TomlArray internalPointsToml = parseResult.getArray("internal_points");
      ArrayList<Translation2d> path = new ArrayList<>();

      for (int i = 0; i < internalPointsToml.size(); i++) {

        TomlTable waypointToml = internalPointsToml.getTable(i);
        Translation2d waypoint =
            new Translation2d(waypointToml.getDouble("x"), waypointToml.getDouble("y"));
        path.add(waypoint);
      }
      TrajectoryConfig trajectoryConfig =
          new TrajectoryConfig(
              parseResult.getDouble("max_velocity"), parseResult.getDouble("max_acceleration"));
      trajectoryConfig.setStartVelocity(parseResult.getDouble("start_velocity"));
      trajectoryConfig.setEndVelocity(parseResult.getDouble("end_velocity"));
      double yawDegrees = parseResult.getDouble("target_yaw");
      Rotation2d target_Yaw = Rotation2d.fromDegrees(yawDegrees);

      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);
      return new PathData(target_Yaw, trajectoryGenerated);

    } catch (Exception error) {
      System.out.println(error.toString());
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(
              DriveConstants.startPose2d,
              DriveConstants.getDefaultInternalWaypoints(),
              DriveConstants.endPose2d,
              DriveConstants.getDefaultTrajectoryConfig());

      return new PathData(inputs.gyroRotation2d, trajectoryGenerated);
    }
  }

  public void resetGyro() {
    io.resetGyro();
  }

  public void rotateRobot() {
    io.move(0, 0, 0.5);
  }

  private Pose2d parseEndPoint(TomlParseResult parseResult, String poseName) {
    TomlTable table = parseResult.getTable(poseName);
    return new Pose2d(
        table.getDouble("x"),
        table.getDouble("y"),
        Rotation2d.fromDegrees(table.getDouble("angle")));
  }

  public void activateHaloRing() {
    holonomicController.setEnabled(true);
  }

  public void deactivateHaloRing() {
    holonomicController.setEnabled(false);
  }

  public void resetHaloRing() {
    xController.reset();
    yController.reset();
    omegaController.reset(inputs.gyroRotation2d.getRadians());
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  public void halt() {
    io.move(0, 0, 0);
  }

  public Rotation2d obtainGyroRotation() {
    return inputs.gyroRotation2d;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    io.registerWith(telemetryService);
    super.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Gyro Rotation", () -> inputs.gyroRotation),
        new Measure("Odometry X", () -> inputs.odometryX),
        new Measure("Odometry Y", () -> inputs.odometryY),
        new Measure("Trajectory Vel", () -> holoInput.velocityMetersPerSecond),
        new Measure("Pose Meters X", () -> holoInput.poseMeters.getX()),
        new Measure("Pose Meters Y", () -> holoInput.poseMeters.getY()),
        new Measure("Traj Acceleration", () -> holoInput.accelerationMetersPerSecondSq),
        new Measure("Holo Output X", () -> holoOutput.vxMetersPerSecond),
        new Measure("Holo Output Y", () -> holoOutput.vyMetersPerSecond));
  }

  @Override
  public void periodic() {
    currentPose = new Pose2d(inputs.odometryX, inputs.odometryY, holoAngle);
    org.littletonrobotics.junction.Logger.recordOutput("Velocity", (currentPose.minus(savedPose)));
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    io.updateSwerve();
  }
}
