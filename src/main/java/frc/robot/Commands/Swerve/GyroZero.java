package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class GyroZero extends InstantCommand{
    
    private SwerveDrive driveTrain;
    
    public GyroZero(){
        this.driveTrain = SwerveDrive.getInstance();
    }

    //TODO:Alex wtf is your gyro code we need this button to work
    @Override
    public void initialize() {
        driveTrain.setGyroAngleAdjustment(0);
    }
    
}
