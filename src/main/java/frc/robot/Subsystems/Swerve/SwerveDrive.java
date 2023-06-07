package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libraries.Swerve.DriveTrain;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.PIDConfig;

public class SwerveDrive extends SubsystemBase {
    DriveTrain driveTrain;
    private static SwerveDrive INSTANCE;
    private double maxSpeed = 5;
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
                new int[] { 1, 3, 5, 7 },
                new int[] { 2, 4, 6, 8 },
                new PIDConfig[] { new PIDConfig(5, .0, 0.02, Double.valueOf(0)),
                        new PIDConfig(5, .0, 0.02, Double.valueOf(0)),
                        new PIDConfig(5, .0, 0.02, Double.valueOf(0)),
                        new PIDConfig(5, .0, 0.02, Double.valueOf(0)) },
                new PIDConfig[] { new PIDConfig(0.1, 0.00, 0.0, 0.08, Double.valueOf(0)),
                        new PIDConfig(0.1, 0.00, 0.0, 0.08, Double.valueOf(0)),
                        new PIDConfig(0.1, 0.00, 0.0, 0.08, Double.valueOf(0)),
                        new PIDConfig(0.1, 0.00, 0.0, 0.08, Double.valueOf(0)) },
                1,
                2048,
                1.0 / 1.0,
                40.0 / 11.0,
                0.038,
                new Translation2d[] { new Translation2d(-1, 1), new Translation2d(1, 1), new Translation2d(-1, -1),
                        new Translation2d(1, -1) },
                new boolean[] { false, false, false, false },
                new boolean[] { false, false, false, false },
                new boolean[] { false, false, false, false },
                new boolean[] { false, false, false, false },
                true,
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                new PIDConfig(0.7, 0, 0, 0, Double.valueOf(0)),
                new PIDConfig(0.7, 0, 0, 0, Double.valueOf(0)));
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        driveTrain.drive(chassisSpeeds, fieldRelative);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.drive(chassisSpeeds, false);
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

    public Pose2d getAllianceRelativePose2d() {
        return driveTrain.getAllianceRelativePose2d();
    }

    public void resetPose2d(Pose2d pose2d) {
        this.driveTrain.resetPose2d(pose2d);
    }

    public void resetAllianceRelativePose2d(Pose2d pose2d) {
        this.driveTrain.resetAllianceRelativePose2d(pose2d);
    }

    public void setGyroAngleAdjustment(double angleAdjustment) {
        this.driveTrain.setGyroAngleAdjustment(angleAdjustment);
    }

    public void setTargetPose2d(Pose2d pose2d) {
        this.driveTrain.setTargetPose2d(pose2d);
    }

    public Rotation2d getGyroRotation() {
        return this.driveTrain.getUnwrappedGyroRotation();
    }

    public void setPoseEstimatorGyroOffset(Rotation2d gyroOffset) {
        this.driveTrain.setPoseEstimatorGyroOffset(gyroOffset);
    }

    public void resetGyro() {
        driveTrain.resetGyro();
    }

    public void setSimulatedGyroAngle(double simulatedAngle) {
        driveTrain.setSimulatedGyroAngle(simulatedAngle);
    }

    public void setPoseEstimatorPose2d(Pose2d pose2d) {
        driveTrain.setPoseEstimatorPose2d(pose2d);
    }
}
