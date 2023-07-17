package frc.robot.Subsystems.arm;

public enum ArmSpeeds {
    
    PLACE_CONE(3),
    EJECT_CONE(5),
    HOLD_CONE(-5),

    PLACE_CUBE(12),
    HOLD_CUBE(-3),

    COLLECT(-3),    
    HOLD_STOW_EITHER(-2),
    NOTHING(0);


    public double roller;
    private ArmSpeeds(double roller) {
        this.roller = roller;
    }
}
