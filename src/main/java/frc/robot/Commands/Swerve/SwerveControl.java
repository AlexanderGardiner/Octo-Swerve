package frc.robot.Commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlMap;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class SwerveControl extends CommandBase{
    SwerveDrive swerveDrive;

    public SwerveControl() {
        this.swerveDrive = SwerveDrive.getInstance();

        this.addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        var leftJoystick = ControlMap.DRIVER_LEFT;
        var rightJoystick = ControlMap.DRIVER_LEFT;

        var xSpeed = 0.0;
        var ySpeed = 0.0;
        var rot = 0.0;

        double xAxis = leftJoystick.getRawAxis(0);
        double yAxis = leftJoystick.getRawAxis(1);
        // Get speeds from joysticks
        xSpeed = -1 * Math.signum(xAxis) * Math.pow(MathUtil.fitDeadband(xAxis, 0.1),2) * swerveDrive.getMaxSpeed();
        ySpeed = -1 * Math.signum(yAxis) * Math.pow(MathUtil.fitDeadband(yAxis, 0.1),2) * swerveDrive.getMaxSpeed();

        // Calculate the deadband
        rot = MathUtil.fitDeadband(-rightJoystick.getRawAxis(4), 0.1) * swerveDrive.getMaxAngularSpeed();

        swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
}
