package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PathHandler.PathHandler;
import frc.robot.subsystems.PathHandler.PathHandler.PathStates;
import java.util.List;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class startPathHandler extends Command {
  Logger logger = LoggerFactory.getLogger(this.getClass());
  private PathHandler pathHandler;
  String[][] pathNames;
  Integer numPieces;
  List<Integer> noteOrder;

  public startPathHandler(
      PathHandler pathHandler, String[][] pathNames, Integer numPieces, List<Integer> noteOrder) {

    this.pathHandler = pathHandler;
    this.pathNames = pathNames;
    this.numPieces = numPieces;
    this.noteOrder = noteOrder;
  }

  @Override
  public void initialize() {
    pathHandler.setOrder(noteOrder);
    pathHandler.setPathNames(pathNames);
    pathHandler.makeTrajectory();
    pathHandler.setNumber(numPieces);
    pathHandler.startHandler();
  }

  @Override
  public boolean isFinished() {
    return pathHandler.getHandlerState() == PathStates.DONE;
  }
}
