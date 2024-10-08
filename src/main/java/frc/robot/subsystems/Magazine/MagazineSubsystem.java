package frc.robot.subsystems.Magazine;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;

import frc.robot.standards.ClosedLoopPosSubsystem;

public class MagazineSubsystem implements ClosedLoopPosSubsystem {
    // Private Variables
    private final MagazineIO io;
    private float speed = 1.0f;

    // Constructor
    public MagazineSubsystem(MagazineIO io) {
        this.io = io;
        zero();
    }

    // Method to intake items
    public void intake() {
        io.spin(speed);
    }

    // Method to stop intake
    public void stopIntake() {
        io.stopSpinning();
    }

    // Method to reset the position (assuming this is required by the interface)
    @Override
    public void zero() {
        // Implement zeroing logic if needed
    }

    // Implement any additional methods required by ClosedLoopPosSubsystem here
}
