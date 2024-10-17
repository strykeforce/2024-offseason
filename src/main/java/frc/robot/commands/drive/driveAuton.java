package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutonCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
// Hello

public class driveAuton extends Command implements AutonCommandInterface {
  private final DriveSubsystem driveSubsystem;
  private Trajectory trajectory;
  private final Timer timer = new Timer();
  private static final Logger logger = LoggerFactory.getLogger(driveAuton.class);
  private Rotation2d robotHeading;
  private boolean lastPath = true;
  private String pathName;
  private boolean resetOdometry;
  private boolean trajectoryGenerated = false;

  public driveAuton(DriveSubsystem driveSubsystem, String pathName, boolean resetOdometry) {

    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.lastPath = lastPath;
    this.resetOdometry = resetOdometry;
    this.pathName = pathName;
    generateTrajectory();
    timer.start();
  }

  public void generateTrajectory() {
    PathData pathData = driveSubsystem.generateTrajectory(pathName);
    trajectory = pathData.trajectory;
    robotHeading = pathData.targetYaw;
  }

  @Override
  public void initialize() {
    driveSubsystem.activateHaloRing();
    Pose2d startingPose = trajectory.getInitialPose();
    if (resetOdometry)
      driveSubsystem.resetOdometry(
          new Pose2d(startingPose.getTranslation(), driveSubsystem.obtainGyroRotation()));
    driveSubsystem.resetHaloRing();
    timer.reset();
    Trajectory.State desiredState = trajectory.sample(timer.get());

    driveSubsystem.holonomicCalculator(desiredState, robotHeading);
  }

  @Override
  public void execute() {
    Trajectory.State desiredState = trajectory.sample(timer.get());
    driveSubsystem.holonomicCalculator(desiredState, robotHeading);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  public void end(boolean interrupeted) {
    System.out.println("Drive auton interupted");
    driveSubsystem.deactivateHaloRing();
    if (!lastPath) {
      driveSubsystem.holonomicCalculator(
          trajectory.sample(trajectory.getTotalTimeSeconds()), robotHeading);
    } else {
      driveSubsystem.move(0, 0, 0);
    }
  }
}
