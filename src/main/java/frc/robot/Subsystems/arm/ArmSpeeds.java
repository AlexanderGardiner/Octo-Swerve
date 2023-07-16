package frc.robot.Subsystems.arm;

public enum ArmSpeeds {
    
    PLACE_CONE(3),
    EJECT_CONE(5),

    COLLECT(-3),
    
    HOLD_CONE(-5),
    HOLD_CUBE(-3),
    HOLD_STOW(-4),
    
    NOTHING(0),
    STOW_WITH_PIECE(0);


    public double roller;
    private ArmSpeeds(double roller) {
        this.roller = roller;
    }
}
