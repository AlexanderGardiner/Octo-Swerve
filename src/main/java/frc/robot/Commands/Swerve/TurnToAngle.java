package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class TurnToAngle extends CommandBase {

    private SwerveDrive swerveDrive;
    private double targetAngle;
    private double tolerance;
    private PIDController pidController = new PIDController(5, 0, 0);

    public TurnToAngle(double targetAngle, double tolerance) {
        this.swerveDrive = SwerveDrive.getInstance();
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        this.pidController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("autoangle", 1);
        swerveDrive.autonomousAngle = true;
    }

    @Override
    public void execute() {
        swerveDrive.drive(new ChassisSpeeds(swerveDrive.previousXSpeed,
                swerveDrive.previousYSpeed,
                pidController.calculate(swerveDrive.getGyroRotation().getRadians(), targetAngle)));
    }

    @Override
    public boolean isFinished() {
        if (MathUtil.isWithinTolerance(targetAngle, swerveDrive.getGyroRotation().getRadians(), tolerance)) {
            swerveDrive.autonomousAngle = false;
            SmartDashboard.putNumber("autoangle", 0);
            swerveDrive.updatePose();
            swerveDrive.setTargetPose2d(swerveDrive.getPose2d());
            return true;
        } else {
            return false;
        }

    }

}
