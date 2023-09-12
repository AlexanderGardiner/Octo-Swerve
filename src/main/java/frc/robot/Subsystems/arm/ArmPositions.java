package frc.robot.Subsystems.arm;

public enum ArmPositions {
    AUTO_ALIGN(0.785, 0.2, 0.35), // taken from original
    STOW(.49, 0.2, .242), // taken from original

    HALF_CONE_PLACE_HIGH(.7, 0.2, .567), // fill in the crossover
    PRE_CONE_PLACE_HIGH(.785, 71, .567), // taken from original
    CONE_PLACE_HIGH(.745, 71, .567), // same as prep but with caligirlsmovedownalittle incorporated

    HALF_CONE_PLACE_MID(.6, 0.2, .619), // fill in the crossover
    PRE_CONE_PLACE_MID(.77, 21.5, .619), // taken from original
    CONE_PLACE_MID(.73, 21.5, .619), // same as prep but with caligirlsmovedownalittle incorporated

    INTAKE_GROUND(.585, 31.4, 0.4),
    INTAKE_SUBSTATION(.78, 0.2, .66),

    CUBE_PLACE_HIGH(.77, 47, .65),

    CUBE_PLACE_MID(.642, 0.2, .493),

    DRIVE_POSITION(.5, 0.2, .31),
    PRE_DRIVE_POSITION(.5, 0.2, .45);

    public double armAngle, extension, wrist;

    ArmPositions(double angle, double extension, double wrist) {
        this.armAngle = angle;
        this.extension = extension;
        this.wrist = wrist;
    }
}