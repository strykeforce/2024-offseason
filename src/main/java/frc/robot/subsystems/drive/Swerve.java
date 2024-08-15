package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import org.strykeforce.gyro.SF_AHRS;
import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;
import org.strykeforce.telemetry.TelemetryService;

public class Swerve implements SwerveIO {

  private final SwerveDrive swerveDrive;
  private TalonFXConfigurator configurator;
  private SF_AHRS ahrs;
  private PoseEstimatorOdometryStrategy odometryStrategy;

  public Swerve() {
    var moduleBuilder =
        new V6TalonSwerveModule.V6Builder()
            .driveGearRatio(DriveConstants.kDriveGearRatio)
            .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
            .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

    V6TalonSwerveModule[] swerveModules = new V6TalonSwerveModule[4];
    Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

    for (int i = 0; i < 4; i++) {
      var azimuthTalon = new TalonSRX(i);
      azimuthTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
      azimuthTalon.configAllSettings(
          DriveConstants.getAzimuthTalonConfig(), Constants.kTalonConfigTimeout);
      azimuthTalon.enableCurrentLimit(true);
      azimuthTalon.enableVoltageCompensation(true);
      azimuthTalon.setNeutralMode(NeutralMode.Coast);

      var driveTalon = new TalonFX(i + 10);
      configurator = driveTalon.getConfigurator();
      configurator.apply(new TalonFXConfiguration()); // factory default
      configurator.apply(DriveConstants.getDriveTalonConfig());
      driveTalon.getSupplyVoltage().setUpdateFrequency(100);
      driveTalon.getSupplyCurrent().setUpdateFrequency(100);
      driveTalon.getClosedLoopReference().setUpdateFrequency(200);

      swerveModules[i] =
          moduleBuilder
              .azimuthTalon(azimuthTalon)
              .driveTalon(driveTalon)
              .wheelLocationMeters(wheelLocations[i])
              .closedLoopUnits(ClosedLoopUnits.VOLTAGE)
              .build();
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }
    ahrs = new SF_AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
    swerveDrive = new SwerveDrive(ahrs, swerveModules);
    swerveDrive.resetGyro();
    swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

    odometryStrategy =
        new PoseEstimatorOdometryStrategy(
            swerveDrive.getHeading(),
            new Pose2d(),
            swerveDrive.getKinematics(),
            VisionConstants.kStateStdDevs,
            VisionConstants.kLocalMeasurementStdDevs,
            VisionConstants.kVisionMeasurementStdDevs,
            getSwerveModulePositions());

    swerveDrive.setOdometry(odometryStrategy);
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public SwerveModule[] getSwerveModules() {
    return swerveDrive.getSwerveModules();
  }

  public Rotation2d getGyroRotation2d() {
    return swerveDrive.getHeading();
  }

  public double getGyroRoll() {
    return ahrs.getRoll();
  }

  public void resetGyro() {
    swerveDrive.resetGyro();
  }

  public void drive(double Xmps, double Ymps, double OmegaRadps) {
    swerveDrive.drive(Xmps, Ymps, OmegaRadps, false);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModule[] swerveModules = getSwerveModules();
    SwerveModulePosition[] temp = {null, null, null, null};
    for (int i = 0; i < 4; ++i) {
      temp[i] = swerveModules[i].getPosition();
    }
    return temp;
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    inputs.odometryX = swerveDrive.getPoseMeters().getX();
    inputs.odometryY = swerveDrive.getPoseMeters().getY();
    inputs.gyroRotation = getGyroRotation2d().getDegrees();
    inputs.odometryRotation2D = swerveDrive.getPoseMeters().getRotation().getDegrees();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    swerveDrive.registerWith(telemetryService);
  }
}
