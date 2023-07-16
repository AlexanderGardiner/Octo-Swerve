package frc.robot.Subsystems.hippo;

public enum HippoPositions {

    INTAKE(0, 0),
    STOW(0, 0),
    PLACE(0, 0);

    
    public double angle, speed;
    HippoPositions(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }
}
