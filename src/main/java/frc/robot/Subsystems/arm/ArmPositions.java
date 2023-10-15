package frc.robot.Subsystems.arm;

public enum ArmPositions {
    AUTO_ALIGN(0.785, 0.2, 0.35), // taken from original
    STOW(.487, 0, .242), // taken from original

    HALF_CONE_PLACE_HIGH(.758, 0.2, .567), // fill in the crossover
    PRE_CONE_PLACE_HIGH(.8, 70.5, .58), // taken from original
    CONE_PLACE_HIGH(.75, 69.2, .582), // same as prep but with caligirlsmovedownalittle incorporated

    HALF_CONE_PLACE_MID(.728, 0.2, .619), // fill in the crossover
    PRE_CONE_PLACE_MID(.76, 23.35, .65), // taken from original
    CONE_PLACE_MID(.728, 17.85, .63), // same as prep but with caligirlsmovedownalittle incorporated

    INTAKE_GROUND(.58, 30.43, 0.44),
    INTAKE_SUBSTATION(.77, 0.2, .66),

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