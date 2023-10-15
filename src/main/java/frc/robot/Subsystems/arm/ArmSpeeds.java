package frc.robot.Subsystems.arm;

public enum ArmSpeeds {

    PLACE_CONE(-2),
    EJECT_CONE(-4),
    HOLD_CONE(4),

    PLACE_CUBE(-7),
    HOLD_CUBE(4),

    COLLECT(10),
    HOLD_STOW_EITHER(10),
    NOTHING(0);

    public double roller;

    private ArmSpeeds(double roller) {
        this.roller = roller;
    }
}
