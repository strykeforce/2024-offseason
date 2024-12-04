package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.PathHandler.PathHandler;
import frc.robot.subsystems.auto.AutonCommandInterface;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import frc.robot.commands.intake.StartIntakeCommand;
import frc.robot.subsystems.Intake.Intake;

public class drivePathHandler extends SequentialCommandGroup implements AutonCommandInterface {
  private Intake intake;
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
      Intake intake,
      String pathname,
      List<Integer> noteOrder,
      int numPieces) {
    addRequirements(driveSubsystem, pathHandler, intake);
    firstPath = new driveAuton(driveSubsystem, pathHandler, "standardAutonStart", true, false);
    addCommands(
        new SequentialCommandGroup(
            new StartIntakeCommand(intake),
            new resetGyro(driveSubsystem),
            firstPath,
            new startPathHandler(pathHandler, pathNames, numPieces, noteOrder)));
  }

  public void generateTrajectory() {}
}
