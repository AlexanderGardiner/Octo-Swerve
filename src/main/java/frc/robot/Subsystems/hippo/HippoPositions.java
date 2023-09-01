package frc.robot.Subsystems.hippo;

public enum HippoPositions {

    INTAKE(0.1, -3),
    STOW(0.45, -0.5),
    PLACE(0.1, 3);

    
    public double angle, speed;
    HippoPositions(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }
}
