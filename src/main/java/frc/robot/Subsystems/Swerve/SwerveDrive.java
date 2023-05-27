package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libraries.Swerve.DriveTrain;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.PIDConfig;

public class SwerveDrive extends SubsystemBase {
    DriveTrain driveTrain;


    
    public SwerveDrive() {
        this.driveTrain = new DriveTrain(MotorType.SparkMax, 
                                         MotorType.TalonFX, 
                                         new int[]{1,3,5,7}, 
                                         new int[]{2,4,6,8}, 
                                         new PIDConfig[]{new PIDConfig(0, 0, 0, 0, Double.valueOf(0)),new PIDConfig(0, 0, 0, 0, Double.valueOf(0)),new PIDConfig(0, 0, 0, 0, Double.valueOf(0)),new PIDConfig(0, 0, 0, 0, Double.valueOf(0))}, 
                                         new PIDConfig[]{new PIDConfig(0, 0, 0, 0, Double.valueOf(0)),new PIDConfig(0, 0, 0, 0, Double.valueOf(0)),new PIDConfig(0, 0, 0, 0, Double.valueOf(0)),new PIDConfig(0, 0, 0, 0, Double.valueOf(0))}, 
          4096, 
         4096, 
                                         1.0/1.0, 
                                         1.0/1.0, 
                             0, 
                                         new Translation2d[]{new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()},
                                         new boolean[]{false, false, false, false},
                                         new boolean[]{false, false, false, false},
                                         new boolean[]{false, false, false, false},
                                         new boolean[]{false, false, false, false});
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        driveTrain.drive(chassisSpeeds, true);
    }

    public void initDefaultCommand() {
        this.setDefaultCommand(null);
    }
}
