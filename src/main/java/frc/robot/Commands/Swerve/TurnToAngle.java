package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class TurnToAngle extends InstantCommand {
    private SwerveDrive swerveDrive;
    private double targetAngle;

    public TurnToAngle(double targetAngle) {
        this.swerveDrive = SwerveDrive.getInstance();
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("autoangle", 1);
        Pose2d targetPose = swerveDrive.getTargetPose2d();
        this.swerveDrive.setTargetPose2d(new Pose2d(targetPose.getX(), targetPose.getY(), new Rotation2d(targetAngle)));
    }


}
