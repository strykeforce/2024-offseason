package frc.robot.subsystems.PathHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

public class PathHandler extends SubsystemBase {
  private DriveSubsystem driveSubsystem;
  private Timer timer = new Timer();
  private boolean handling = true;
  private PathStates curState = PathStates.DONE;
  private boolean canShoot = false;
  private boolean noteHeld = false;
  private int numPieces = 3;
  private Rotation2d robotHeading;
  private Trajectory curTrajectory;
  private ArrayList<Integer> noteOrder;
  private String pathName[][];
  private PathData nextPath;
  private Integer lastNote;
  private Integer nextNote;
  private PathData[][] paths = new PathData[6][6];

  public PathHandler(DriveSubsystem driveSubsystem) {}

  public void makeTrajectory() {
    noteOrder.add(0);

    Set<Integer> singleNotes = new HashSet<Integer>(noteOrder);

    for (int i : singleNotes) {
      for (int j : singleNotes) {
        if (i != j) {
          paths[i][j] = driveSubsystem.generateTrajectory(pathName[i][j]);
        }
      }
    }
    noteOrder.remove(noteOrder.indexOf(0));
  }

  public void setState(PathStates state) {
    curState = state;
  }

  public void startHandler() {
    timer.reset();
    timer.restart();
    curState = PathStates.DRIVE_NOTE;
    handling = true;
  }

  public void stopHandlerl() {
    timer.reset();
    timer.stop();
    curState = PathStates.DONE;
    handling = false;
  }

  public void startNewPath(PathData path) {
    curTrajectory = path.trajectory;
    robotHeading = path.targetYaw;
    driveSubsystem.activateHaloRing();
    driveSubsystem.resetHaloRing();
    timer.reset();
    timer.start();

    driveSubsystem.holonomicCalculator(curTrajectory.sample(timer.get()), robotHeading);
  }

  public void getNextPath() {
    lastNote = noteOrder.get(0);
    noteOrder.remove(0);
    nextNote = noteOrder.get(0);
  }

  @Override
  public void periodic() {
    if (handling) {
      switch (curState) {
        case DRIVE_NOTE:
          if (numPieces != 0) {
            driveSubsystem.holonomicCalculator(curTrajectory.sample(timer.get()), robotHeading);
            if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {
              if (noteHeld) {
                curState = PathStates.DRIVE_SHOOT;
              } else {
                numPieces--;
                startNewPath(paths[lastNote][nextNote]);
              }
            }
          } else {
            curState = PathStates.DONE;
          }
          break;
        case DRIVE_SHOOT:
          if (canShoot) {
            startNewPath(paths[lastNote][0]);
            driveSubsystem.holonomicCalculator(curTrajectory.sample(timer.get()), robotHeading);
            if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {
              curState = PathStates.SHOOT;
            }
          } else {
            startNewPath(paths[lastNote][nextNote]);
            curState = PathStates.DRIVE_NOTE;
          }
          break;
        case SHOOT:
          // We can't do any of this yet so I'm not even going to start
          // putting logic in here because I have no idea what to do about it
          break;
        case DONE:
          handling = false;
          break;
      }
    }
  }

  private enum PathStates {
    DRIVE_NOTE,
    DRIVE_SHOOT,
    SHOOT,
    DONE
  }
}
