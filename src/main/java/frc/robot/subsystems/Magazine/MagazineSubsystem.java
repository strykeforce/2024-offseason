package frc.robot.subsystems.Magazine;

import org.strykeforce.telemetry.TelemetryService;

public class MagazineSubsystem implements MagazineIO {
  private final MagazineIOFX magazineIOFX;

  public MagazineSubsystem() {
    magazineIOFX = new MagazineIOFX();
    setVelocity(0);
  }

  @Override
  public boolean spin(double speed) {
    return magazineIOFX.spin(speed);
  }

  @Override
  public void stopSpinning() {
    magazineIOFX.stopSpinning();
  }

  @Override
  public double atSpeed() {
    return magazineIOFX.atSpeed();
  }

  public void setVelocity(double speed) {
    magazineIOFX.spin(speed);
  }

  @Override
  public void updateInputs() {
    magazineIOFX.updateInputs();
  }

  @Override
  public boolean isBeamBroken() {
    return magazineIOFX.isBeamBroken();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    magazineIOFX.registerWith(telemetryService);
  }
}
