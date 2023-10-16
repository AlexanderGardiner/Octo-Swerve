package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class TurnToAngle extends CommandBase {
    //TODO:This whole file is broken
    private SwerveDrive swerveDrive;
    private double targetAngle;
    private PIDController pidController = new PIDController(6, 0, 0);

    public TurnToAngle(double targetAngle) {
        this.swerveDrive = SwerveDrive.getInstance();
        this.targetAngle = targetAngle;
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
        return false;

    }

}
