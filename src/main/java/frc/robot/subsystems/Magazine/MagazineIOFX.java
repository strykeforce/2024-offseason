package frc.robot.subsystems.Magazine;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

/** Implementation of the MagazineIO interface using TalonFX motors. */
public class MagazineIOFX implements MagazineIO {
  private Logger logger;
  private final TalonFX talonFX;
  private IOInputs inputs;

  // FX Access objects
  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Boolean> currBeambreak;
  StatusSignal<Double> currVelocity;
  private StatusSignal<ReverseLimitValue> reverseLimitSwitch;

  // Constructor to initialize the TalonFX motor controller
  public MagazineIOFX() {
    final IOInputsAutoLogged inputs = new IOInputsAutoLogged();
    logger = LoggerFactory.getLogger(this.getClass());
    talonFX = new TalonFX(magazineConstants.kFxId);
    reverseLimitSwitch = talonFX.getReverseLimit();
    currVelocity = talonFX.getVelocity();
    reverseLimitSwitch = talonFX.getReverseLimit();
    // currBeambreak =
  }

  public double atSpeed() {
    return inputs.velocity;
  }

  public void updateInputs() {
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.beambreak = reverseLimitSwitch.refresh().getValue().value == 1;
  }

  public boolean spin(double inputSpeed) {
    // Optional: Safety check to limit speed range
    if (inputSpeed < -1.0 || inputSpeed > 1.0) {
      return false;
    }
    talonFX.set(inputSpeed);
    return true;
  }

  public boolean atSpeed(double targetSpeed) {
    double currentSpeed = currVelocity.refresh().getValue();
    return Math.abs(currentSpeed - targetSpeed) < 0.01; // tolerance
  }

  public boolean isBeamBroken() {
    return currBeambreak.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }
}
