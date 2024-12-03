package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.PathHandler.PathHandler;
import frc.robot.subsystems.auto.AutonCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class drivePathHandler extends SequentialCommandGroup implements AutonCommandInterface {
  private DriveSubsystem driveSubsystem;
  private PathHandler pathHandler;
  driveAuton midStartToFirst;
  driveAuton FirstToShoot;
  String[][] pathNames = AutonConstants.pathMatrix;
  Integer numPieces;
  driveAuton firstPath;
  ArrayList<Integer> noteOrder;

  public drivePathHandler(
      DriveSubsystem driveSubsystem,
      PathHandler pathHandler,
      String pathname,
      List<Integer> noteOrder,
      int numPieces) {
    addRequirements(driveSubsystem, pathHandler);
    firstPath = new driveAuton(driveSubsystem, pathHandler, "standardAutonStart", true, false);
    addCommands(
        new SequentialCommandGroup(
            new resetGyro(driveSubsystem),
            firstPath,
            new startPathHandler(pathHandler, pathNames, numPieces, noteOrder)));
  }

  public void generateTrajectory() {}
}
