package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class TurnToAngle extends InstantCommand {
    private SwerveDrive swerveDrive;
    private double targetAngle;

    //TODO:inverts on side change
    public TurnToAngle(double targetAngle) {
        this.swerveDrive = SwerveDrive.getInstance();
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = swerveDrive.getTargetPose2d();
        if (DriverStation.getAlliance()==DriverStation.Alliance.Red) {
            this.swerveDrive.setTargetPose2d(new Pose2d(targetPose.getX(), targetPose.getY(), new Rotation2d(Math.PI-targetAngle)));
        } else {
            this.swerveDrive.setTargetPose2d(new Pose2d(targetPose.getX(), targetPose.getY(), new Rotation2d(targetAngle)));
        }
        
    }


}

