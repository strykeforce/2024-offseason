package frc.robot.subsystems.exiter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.ExiterConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class ExiterIOFX implements ExiterIO {
  // private objects
  private Logger logger;
  private TalonFX talonFxLeft;
  private TalonFX talonFxRight;

  private double setpointLeft;
  private double setpointRight;

  // FX Access objects
  TalonFXConfigurator configuratorLeft;
  TalonFXConfigurator configuratorRight;
  StatusSignal<Double> currLeftVelocity;
  StatusSignal<Double> currRightVelocity;
  private MotionMagicVelocityVoltage velocityRequestLeft =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false).withSlot(0);
  private MotionMagicVelocityVoltage velocityRequestRight =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false).withSlot(0);

  public ExiterIOFX() {
    logger = LoggerFactory.getLogger(this.getClass());
    talonFxLeft = new TalonFX(ExiterConstants.kFxIDLeft);
    talonFxRight = new TalonFX(ExiterConstants.kFxIDRight);

    // Config controller
    configuratorLeft = talonFxLeft.getConfigurator();
    configuratorRight = talonFxRight.getConfigurator();
    configuratorLeft.apply(ExiterConstants.getLeftFXConfig());
    configuratorRight.apply(ExiterConstants.getRightFXConfig());

    // Attach status signals
    currLeftVelocity = talonFxLeft.getVelocity();
    currRightVelocity = talonFxRight.getVelocity();
  }

  @Override
  public void updateInputs(ExiterIOInputs inputs) {
    inputs.velocityLeft = currLeftVelocity.refresh().getValue();
    inputs.velocityRight = currRightVelocity.refresh().getValue();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFxLeft, true);
    telemetryService.register(talonFxRight, true);
  }

  @Override
  public void setSpeed(double leftSpeed, double rightSpeed) {
    talonFxLeft.setControl(velocityRequestLeft.withVelocity(leftSpeed));
    talonFxRight.setControl(velocityRequestRight.withVelocity(rightSpeed));
    setpointLeft = leftSpeed;
    setpointRight = rightSpeed;
  }
}
