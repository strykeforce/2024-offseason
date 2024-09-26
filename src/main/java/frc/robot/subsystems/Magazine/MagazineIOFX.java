package frc.robot.subsystems.Magazine;
import com.ctre.phoenix6.hardware.TalonFX;

public class MagazineIOFX implements MagazineIO{
    //objects
    private TalonFX talonFX;
    //storage variables
    private double speed;
    
    public void spin(double importspeed){
        speed = importspeed;
        talonFX.stopMotor();
        talonFX.set(speed);
    }
    public void stopSpinning(){
        talonFX.stopMotor();
    }
    public double getSpeed(){
        return talonFX.get();
    }
    public boolean beambreak(){
        return false;
    }
    }
