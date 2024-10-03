package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;

import frc.robot.constants.IntakeConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class IntakeIOFX implements IntakeIO{
    // private objects
  private Logger logger;
  private TalonFX talonFx;

  // storage variables
  private final double absSensorInitial;
  
  // FX Access objects
  TalonFXConfigurator configurator;
  StatusSignal<Double> currVelocity;
  StatusSignal<ForwardLimitValue> currFwdLimit;

  public IntakeIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFx = new TalonFX(IntakeConstants.IntakeFxId);
    absSensorInitial =
        talonFx.getPosition().getValue(); // relative encoder starts up as absolute position offset

    // Config controller
    configurator = talonFx.getConfigurator();
    configurator.apply(new TalonFXConfiguration()); // Factory default motor controller
    configurator.apply(IntakeConstants.getFXConfig());

    // Attach status signals
    currVelocity = talonFx.getVelocity();
    currFwdLimit = talonFx.getForwardLimit();
  }

  @Override
  public void updateInputs(ExampleIOInputs inputs) {
    inputs.velocity = currVelocity.refresh().getValue();
    inputs.fwdBeamOpen = currFwdLimit.refresh().getValue().value == 1;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFx, true);
  }

  public void SetSpeed(double speed)
  {
    talonFx.set(speed);
  }
}
