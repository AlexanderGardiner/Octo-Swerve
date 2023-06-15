package frc.robot.Commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlMap;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class SwerveControl extends CommandBase {
    SwerveDrive swerveDrive;

    /**
     * Creates a new swerve control command and adds the swervedrive subsystme as a
     * requirement
     */
    public SwerveControl() {
        this.swerveDrive = SwerveDrive.getInstance();

        this.addRequirements(swerveDrive);
    }

    /**
     * Runs every robot loop while the command is running
     * <ul>
     * <li>Gets speeds from the joysticks</li>
     * <li>Applies deadbands and square the speeds to give better control</li>
     * <li>Flips the speeds if on the opposite alliance</li>
     * <li>Drives the robot with the calculated speeds</li>
     * </ul>
     */
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
        xSpeed = -1 * Math.signum(yAxis) * Math.pow(MathUtil.fitDeadband(yAxis, 0.05), 2) * swerveDrive.getMaxSpeed();
        ySpeed = -1 * Math.signum(xAxis) * Math.pow(MathUtil.fitDeadband(xAxis, 0.05), 2) * swerveDrive.getMaxSpeed();

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            xSpeed = xSpeed * -1;
            ySpeed = ySpeed * -1;
        }

        // Calculate the deadband
        rot = MathUtil.fitDeadband(-leftJoystick.getRawAxis(4), 0.05) * swerveDrive.getMaxAngularSpeed();

        // Drive
        swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, rot), true);
    }
}
