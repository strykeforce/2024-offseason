package frc.robot.subsystems.drive;
import java.util.Set;

import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.controllers.FlyskyJoystick;

public class DriveSubsystem extends MeasurableSubsystem {

  private final SwerveIO io;
  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private final Swerve swerve;
  private final SwerveDrive swerveDrive;
  private final FlyskyJoystick controllerFlyskyJoystick;

  public DriveSubsystem(SwerveIO io){
    this.io = io;
    this.swerve = new Swerve();
    this.swerveDrive = swerve.getSwerveDrive();
    
  }



  @Override
  public Set<Measure> getMeasures() {
      // TODO Auto-generated method stub
      return Set.of(
        new Measure("Gyro roll", () -> swerve.getGyroRoll())
      );
  }

}
