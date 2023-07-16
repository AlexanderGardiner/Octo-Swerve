package frc.robot.Subsystems.arm;

public enum ArmPositions {
    // TODO: all of this is probably bad now, bruh moment
    AUTO_ALIGN(0, 0, 0),
    STOW(0, 0, 0),
    HALF_CONE_PLACE_HIGH(0,0,0),
    PRE_CONE_PLACE_HIGH(0, 0, 0),
    HALF_CONE_PLACE_MID(0,0,0),
    PRE_CONE_PLACE_MID(0,0,0),
    INTAKE_GROUND(0,0,0),
    INTAKE_SUBSTATION(0,0,0), 
    CUBE_PLACE_HIGH(0,0,0), 
    CUBE_PLACE_MID(0,0,0),
    DRIVE_POSITION(0,0,0),
    PRE_DRIVE_POSITION(0,0,0);

    public double armAngle, extension, wrist;

    ArmPositions(double angle, double extension, double wrist) {
        this.armAngle = angle;
        this.extension = extension;
        this.wrist = wrist;
    }
}