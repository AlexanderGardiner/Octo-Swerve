package frc.robot.Commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class GyroZero extends InstantCommand{


    //TODO:Alex wtf is your gyro code we need this button to work
    @Override
    public void initialize() {
        SwerveDrive.getInstance().resetGyro();
        SwerveDrive.getInstance().updatePose();
        SwerveDrive.getInstance().setTargetPose2d(SwerveDrive.getInstance().getPose2d());
    }
    
}
