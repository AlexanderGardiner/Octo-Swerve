package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.drivetrain.SwerveDrive;


public class TogglePidRotation extends InstantCommand{


    private SwerveDrive swerveDrive;

    public TogglePidRotation() {
        this.swerveDrive = SwerveDrive.getInstance();
    }
    @Override
    public void initialize() {
        if (swerveDrive.getPidRotation()) {
            swerveDrive.setPidRotation(false);
        } else {
            swerveDrive.setTargetPose2d(swerveDrive.getPose2d());
            swerveDrive.setPidRotation(true);
            
        }
    }
    
}
