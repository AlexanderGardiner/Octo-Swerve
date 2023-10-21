package frc.robot.Subsystems.hippo;

public enum HippoPositions {

    INTAKE(0.1, 6),
    STOW(0.45, 0.5),
    PLACE(0.1, -1.5),
    YEET(0.4, 11);

    public double angle, speed;

    HippoPositions(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }
}
