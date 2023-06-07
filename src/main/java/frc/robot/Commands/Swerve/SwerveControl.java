package frc.robot.Commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlMap;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class SwerveControl extends CommandBase {
    SwerveDrive swerveDrive;

    public SwerveControl() {
        this.swerveDrive = SwerveDrive.getInstance();

        this.addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var leftJoystick = ControlMap.DRIVER_LEFT;
        var rightJoystick = ControlMap.DRIVER_RIGHT;

        var xSpeed = 0.0;
        var ySpeed = 0.0;
        var rot = 0.0;

        double xAxis = leftJoystick.getRawAxis(0);
        double yAxis = leftJoystick.getRawAxis(1);

        // Get speeds from joysticks
        xSpeed = -1 * Math.signum(yAxis) * Math.pow(MathUtil.fitDeadband(yAxis, 0.01), 2) * swerveDrive.getMaxSpeed();
        ySpeed = -1 * Math.signum(xAxis) * Math.pow(MathUtil.fitDeadband(xAxis, 0.01), 2) * swerveDrive.getMaxSpeed();

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            xSpeed = xSpeed * -1;
            ySpeed = ySpeed * -1;
        }

        // Calculate the deadband
        rot = MathUtil.fitDeadband(-rightJoystick.getRawAxis(0), 0.01) * swerveDrive.getMaxAngularSpeed();

        // Drive
        swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, rot), true);
    }
}
