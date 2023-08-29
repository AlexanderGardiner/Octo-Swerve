package frc.robot.Commands.Swerve;

import com.fasterxml.jackson.databind.deser.std.DateDeserializers.DateDeserializer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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















        double xAxis = -1 * leftJoystick.getRawAxis(0);
        double yAxis = -1 * leftJoystick.getRawAxis(1);

        
        


        /**
         *  Code deadband code here
         *  The joysticks may have some small amount of input when not being touched
         *  we use deadbands to remove any input from below a certain threshold
         */ 

        double deadbandedXAxis;
        double deadbandedYAxis;

        if(Math.abs(xAxis) < 0.1) {
            deadbandedXAxis = 0;
        } else {
            deadbandedXAxis = xAxis;
        }


        if(Math.abs(yAxis) < 0.1) {
            deadbandedYAxis = 0;
        } else {
            deadbandedYAxis = yAxis; 
        }

        
        
        if(deadbandedXAxis>0) {
            deadbandedXAxis=(deadbandedXAxis-0.1)* (10/9);
        }else if(deadbandedXAxis<0){
            deadbandedXAxis=(deadbandedXAxis+0.1)* (10/9);
        }

        if(deadbandedYAxis>0) {
            deadbandedYAxis=(deadbandedYAxis-0.1)* (10/9);
        
        }else if (deadbandedYAxis<0){
            deadbandedYAxis=(deadbandedYAxis+0.1)* (10/9);
        }

        SmartDashboard.putNumber("x", deadbandedXAxis);
        SmartDashboard.putNumber("y", deadbandedYAxis);



















        

        // Get speeds from joysticks
        xSpeed = deadbandedXAxis * swerveDrive.getMaxSpeed();
        ySpeed = deadbandedYAxis * swerveDrive.getMaxSpeed();

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            xSpeed = xSpeed * -1;
            ySpeed = ySpeed * -1;
        }

        // Calculate the deadband
        rot = MathUtil.fitDeadband(-rightJoystick.getRawAxis(0), 0.05) * swerveDrive.getMaxAngularSpeed();

       
        // Drive
        swerveDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, rot), true);
    }
}
