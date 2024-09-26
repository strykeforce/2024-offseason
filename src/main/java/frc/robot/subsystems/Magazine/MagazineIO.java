package frc.robot.subsystems.Magazine;

public interface MagazineIO {

    //spins the motor at a set speed
    public void spin(double speed);
    
    //stops the motor
    public void stopSpinning();

    //returns if the motor is running
    public double getSpeed();

    public boolean beambreak();
}