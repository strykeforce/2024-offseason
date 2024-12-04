package frc.robot.subsystems.PathHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathData;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;

public class PathHandler extends SubsystemBase {
  private DriveSubsystem driveSubsystem;
  private Timer timer = new Timer();
  private Intake intake;
  private boolean handling = true;
  private PathStates curState = PathStates.DONE;
  private boolean canShoot = true;
  private int numPieces;
  private Rotation2d robotHeading;
  private Trajectory curTrajectory;
  private ArrayList<Integer> noteOrder;
  private String pathName[][];
  private PathData nextPath;
  private Integer lastNote;
  private Integer nextNote;
  private PathData[][] paths = new PathData[4][4];
  private Integer targetNote = 1;

  public PathHandler(DriveSubsystem driveSubsystem, Intake intake) {
    this.driveSubsystem = driveSubsystem;
    this.targetNote = targetNote;
    this.intake = intake;
  }

  public void setState(PathStates state) {
    curState = state;
  }

  public void setOrder(List<Integer> list) {
    noteOrder = new ArrayList<>(list);
    lastNote = noteOrder.get(0);
    nextNote = noteOrder.get(1);
  }

  public void setNumber(Integer numPieces) {
    this.numPieces = numPieces;
  }

  public void setPathNames(String[][] pathNames) {
    this.pathName = pathNames;
  }

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

  public void startHandler() {
    targetNote = 1;
    timer.reset();
    timer.restart();
    curState = PathStates.DRIVE_NOTE;
    handling = true;
    if (intake.getState() != IntakeState.Running) {
      startNewPath(paths[lastNote][0]);
      curState = PathStates.DRIVE_SHOOT;
    } else {
      numPieces--;
      intake.StartIntake();
      lastNote = noteOrder.get(0);
      nextNote = noteOrder.get(1);
      noteOrder.remove(0);
      startNewPath(paths[lastNote][nextNote]);
    }
  }

  public void stopHandlerl() {
    timer.reset();
    timer.stop();
    curState = PathStates.DONE;
    handling = false;
  }

  public void startNewPath(PathData path) {
    intake.StartIntake();
    targetNote = nextNote;
    curTrajectory = path.trajectory;
    robotHeading = path.targetYaw;
    driveSubsystem.activateHaloRing();
    driveSubsystem.resetHaloRing();
    timer.reset();
    timer.reset();
    timer.start();
    driveSubsystem.holonomicCalculator(curTrajectory.sample(timer.get()), robotHeading);
  }

  public void getNextPath() {
    lastNote = noteOrder.get(0);
    noteOrder.remove(0);
    nextNote = noteOrder.get(0);
  }

  public PathStates getHandlerState() {
    return curState;
  }

  @Override
  public void periodic() {
    org.littletonrobotics.junction.Logger.recordOutput("PathHandler", (curState));
    org.littletonrobotics.junction.Logger.recordOutput("Target Note", targetNote);
    if (handling) {
      switch (curState) {
        case DRIVE_NOTE:
          if (numPieces > 0) {
            driveSubsystem.holonomicCalculator(curTrajectory.sample(timer.get()), robotHeading);
            if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {
              if (intake.getState() != IntakeState.Running) {
                lastNote = noteOrder.get(0);
                targetNote = 0;
                startNewPath(paths[lastNote][0]);
                curState = PathStates.DRIVE_SHOOT;
                break;
              } else {
                numPieces--;
                if (noteOrder.size() > 1) {
                  lastNote = noteOrder.get(0);
                  nextNote = noteOrder.get(1);
                  noteOrder.remove(0);
                  startNewPath(paths[lastNote][nextNote]);
                } else {
                  lastNote = nextNote;
                  startNewPath(paths[lastNote][nextNote]);
                  noteOrder.remove(0);
                }
              }
            }
          } else {
            curState = PathStates.DONE;
          }
          break;
        case DRIVE_SHOOT:
          if (canShoot) {
            driveSubsystem.holonomicCalculator(curTrajectory.sample(timer.get()), robotHeading);
            if (timer.hasElapsed(curTrajectory.getTotalTimeSeconds())) {

              curState = PathStates.SHOOT;
            }
          } else {
            //  startNewPath(paths[0][noteOrder.get(0)]);
            // curState = PathStates.DRIVE_NOTE;
          }
          break;
        case SHOOT:
          // We can't do any of this yet so I'm not even going to start
          // putting logic in here because I have no idea what to do about it
          if (numPieces > 1) {
            lastNote = noteOrder.get(0);
            nextNote = noteOrder.get(1);
            noteOrder.remove(0);
            numPieces--;

            startNewPath(paths[0][nextNote]);
            curState = PathStates.DRIVE_NOTE;
          } else {
            curState = PathStates.DONE;
          }
          break;
        case DONE:
          handling = false;
          break;
      }
    }
  }

  public enum PathStates {
    DRIVE_NOTE,
    DRIVE_SHOOT,
    SHOOT,
    DONE
  }
}
