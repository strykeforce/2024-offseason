package frc.robot.subsystems.exiter;

import frc.robot.constants.ExiterConstants;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ExiterSubsystem extends MeasurableSubsystem {
  // Private Variables
  private final ExiterIO io;
  private final ExiterIOInputsAutoLogged inputs = new ExiterIOInputsAutoLogged();

  private double setpointLeft;
  private double setpointRight;

  // Constructor
  public ExiterSubsystem(ExiterIO io) {
    this.io = io;
  }

  // Periodic Function
  @Override
  public void periodic() {
    // Read Inputs
    io.updateInputs(inputs);
    org.littletonrobotics.junction.Logger.processInputs("ExiterInputs", inputs);

    // Log Outputs
    // Logger.recordOutput("Exiter/curState", curState.ordinal());
    Logger.recordOutput("Exiter/setpointLeft", setpointLeft);
    Logger.recordOutput("Exiter/setpointRight", setpointRight);
    Logger.recordOutput("Exiter/atSpeed", atSpeed());
  }

  // Grapher
  @Override
  public void registerWith(TelemetryService telemetryService) {

    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }

  public Set<Measure> getMeasures() {
    return Set.of();
  }

  public double getLeftSpeed() {
    return setpointLeft;
  }

  public double getRightSpeed() {
    return setpointRight;
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    setpointLeft = leftSpeed;
    setpointRight = rightSpeed;
    io.setSpeed(leftSpeed, rightSpeed);
  }

  public boolean atSpeed() {
    return Math.abs(setpointLeft - inputs.velocityLeft) <= ExiterConstants.kCloseEnough
        && Math.abs(setpointRight - inputs.velocityRight) <= ExiterConstants.kCloseEnough;
  }
}
