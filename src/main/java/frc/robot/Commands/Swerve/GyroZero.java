package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.drivetrain.SwerveDrive;
import frc.robot.Subsystems.light.Animations;
import frc.robot.Subsystems.light.Light;

public class GyroZero extends InstantCommand{


    private SwerveDrive swerveDrive;

    public GyroZero() {
        this.swerveDrive = SwerveDrive.getInstance();
    }
    @Override
    public void initialize() {
        Light.getInstance().setAnimation(Animations.BOOT_COMPLETE);
        swerveDrive.resetGyro();
        if (DriverStation.getAlliance()==DriverStation.Alliance.Red) {
            swerveDrive.setGyroAngleAdjustment(180);
        } else {
            swerveDrive.setGyroAngleAdjustment(0);
        }
       
        swerveDrive.updatePose();
        swerveDrive.setTargetPose2d(SwerveDrive.getInstance().getPose2d());
    }
    
}
