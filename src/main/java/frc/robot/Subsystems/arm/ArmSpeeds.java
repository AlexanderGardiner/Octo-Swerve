package frc.robot.Subsystems.arm;

public enum ArmSpeeds {
    
    STOW_WITH_PIECE(0);


    public double roller;
    private ArmSpeeds(double roller) {
        this.roller = roller;
    }
}
