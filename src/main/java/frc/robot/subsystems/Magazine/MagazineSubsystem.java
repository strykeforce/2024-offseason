package frc.robot.subsystems.Magazine;

import java.util.Set;

import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import frc.robot.standards.ClosedLoopPosSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class MagazineSubsystem extends MeasurableSubsystem{
  private final MagazineIOFX magazineIOFX;
  private final IOInputsAutoLogged inputs = new IOInputsAutoLogged();
  private int testcount = 5;

  public MagazineSubsystem() {
    magazineIOFX = new MagazineIOFX();
  }

  public boolean setSpin(double speed) {
    return magazineIOFX.spin(speed);
  }

  public double atSpeed() {
    return magazineIOFX.atSpeed();
  }

  public boolean isBeamBroken() {
        for(int i = 0; i < testcount; i++){
        if (isBeamBroken()){
            return true;
        }
    }
    return false;
  }

    public void Periodic(){
      magazineIOFX.updateInputs();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    magazineIOFX.registerWith(telemetryService);
  }
}
