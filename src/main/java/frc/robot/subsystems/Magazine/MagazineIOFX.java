package frc.robot.subsystems.Magazine;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import ch.qos.logback.classic.spi.Configurator;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.constants.ExampleConstants;

/**
 * Implementation of the MagazineIO interface using TalonFX motors.
 */
public class MagazineIOFX implements MagazineIO {
      // private objects
  private Logger logger;
  private final TalonFX talonFX;

  // storage variables
  //ADD MORE LATER 

  // FX Access objects
  TalonFXConfigurator configurator;
  private MotionMagicDutyCycle positionRequest =
      new MotionMagicDutyCycle(0).withEnableFOC(false).withFeedForward(0).withSlot(0);
  StatusSignal<Boolean> currBeambreak;
  StatusSignal<Double> currVelocity;
    private StatusSignal<ReverseLimitValue> reverseLimitSwitch;
    // Constructor to initialize the TalonFX motor controller
    public MagazineIOFX() {
        logger = LoggerFactory.getLogger(this.getClass());
    talonFX = new TalonFX(ExampleConstants.kExampleFxId);
        reverseLimitSwitch = talonFX.getReverseLimit();
        currVelocity = talonFX.getVelocity();
    }
    
    /**
     * Spins the motor at the specified speed.
     * the speed at which to spin the motor
     */
    public void updateInputs(IOInputs inputs){
        inputs.velocity = currVelocity.refresh().getValue();
        inputs.beambreak = currBeambreak.refresh().getValue();
    }

    public boolean spin(double importspeed) {
        // Optional: Safety check to limit speed range
        if (importspeed < -1.0 || importspeed > 1.0) {
           return false;
        }
        talonFX.set(importspeed);
        return true;
    }
    
    /**
     * Stops the motor.
     */
    public void stopSpinning() {
        talonFX.stopMotor();
    }
    
    /**
     * Returns the current speed of the motor.
     * @return the speed of the motor
     */
    public double getSpeed() {
        return talonFX.get();
    }

    public boolean atSpeed(){
        double currentSpeed = talonFX.getVelocity();
        double targetSpeed = talonFX.get(); // Example target speed in native units per 100ms

        if (currentSpeed - targetSpeed == 0) {
    
            // Motor is considered to be at speed
    
        } 
    }
    
    /**
     * Checks if the beam break sensor is triggered.
     * @return true if the beam is broken, false otherwise
     */
    public double isBeamBroken() {
        //currBeambreak = (GET VALUE HERE)
        return 0;
    }
    @Override
      public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(talonFX, true);
  }
}
