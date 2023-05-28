package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libraries.Swerve.DriveTrain;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.PIDConfig;

public class SwerveDrive extends SubsystemBase {
    DriveTrain driveTrain;
    private static SwerveDrive INSTANCE;
    private double maxSpeed = 10;
    private double maxAngularSpeed = Math.PI;

    public static SwerveDrive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive();
        }
        return INSTANCE;
    }

    
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
                             0.05, 
                                         new Translation2d[]{new Translation2d(1,1), new Translation2d(1,-1), new Translation2d(-1,1), new Translation2d(-1,-1)},
                                         new boolean[]{false, false, false, false},
                                         new boolean[]{false, false, false, false},
                                         new boolean[]{false, false, false, false},
                                         new boolean[]{false, false, false, false},
                                         true);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        driveTrain.drive(chassisSpeeds, true);
    }

    public void initDefaultCommand() {
        this.setDefaultCommand(null);
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxSpeed() {
        return this.maxSpeed;
    }

    public void setMaxAngularSpeed(double maxAngularSpeed) {
        this.maxAngularSpeed = maxAngularSpeed;
    }

    public double getMaxAngularSpeed() {
        return this.maxAngularSpeed;
    }

    public Pose2d getPose2d() {
        return driveTrain.getPose2d();
    }
}
