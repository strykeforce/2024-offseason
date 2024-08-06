package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import org.strykeforce.gyro.SF_AHRS;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.V6TalonSwerveModule;
import org.strykeforce.swerve.V6TalonSwerveModule.ClosedLoopUnits;

public class Swerve implements SwerveIO {

  private SwerveDrive swerveDrive;
  private TalonFXConfigurator configurator;
  private SF_AHRS ahrs;

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
  }
}
