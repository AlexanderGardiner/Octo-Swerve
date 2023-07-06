package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Swerve.DriveTrain;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.PIDConfig;

public class SwerveDrive extends SubsystemBase {
    DriveTrain driveTrain;
    private static SwerveDrive INSTANCE;
    private double maxSpeed = 4;
    private double maxAngularSpeed = Math.PI;

    /**
     * Gets the current instance of the swerve drive subsystem if it doesn't exist
     * 
     * @return The current instance of swerve drive
     */
    public static SwerveDrive getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDrive();
        }
        return INSTANCE;
    }

    /**
     * Creates a new swerve drive using the drivetrain class
     */
    public SwerveDrive() {
        this.driveTrain = new DriveTrain(
                MotorType.SparkMax,
                MotorType.TalonFX,
                new int[] { MotorIDs.FRONT_LEFT_STEER, MotorIDs.FRONT_RIGHT_STEER, MotorIDs.BACK_LEFT_STEER, MotorIDs.BACK_RIGHT_STEER},
                new int[] { MotorIDs.FRONT_LEFT_DRIVE, MotorIDs.FRONT_RIGHT_DRIVE, MotorIDs.BACK_LEFT_DRIVE, MotorIDs.BACK_RIGHT_DRIVE},
                new PIDConfig[] { new PIDConfig(5, .0, 0.02, Double.valueOf(0)),
                        new PIDConfig(5, .0, 0.02, Double.valueOf(0)),
                        new PIDConfig(5, .0, 0.02, Double.valueOf(0)),
                        new PIDConfig(5, .0, 0.02, Double.valueOf(0)) },
                new PIDConfig[] { new PIDConfig(0.2, 0.0, 0.0, 0.045, Double.valueOf(0)),
                        new PIDConfig(0.2, 0.00, 0.0, 0.045, Double.valueOf(0)),
                        new PIDConfig(0.2, 0.00, 0.0, 0.045, Double.valueOf(0)),
                        new PIDConfig(0.2, 0.00, 0.0, 0.045, Double.valueOf(0)) },
                1,
                2048,
                1.0 / 1.0,
                40.0 / 11.0,
                0.05,
                new Translation2d[] { new Translation2d(-0.3175, 0.3175), new Translation2d(0.3175, 0.3175),
                        new Translation2d(-0.3175, -0.3175),
                        new Translation2d(0.3175, -0.3175) },
                new boolean[] { false, false, false, false },
                new boolean[] { false, false, false, false },
                new boolean[] { false, false, false, false },
                new boolean[] { false, false, false, false },
                true,
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                new PIDConfig(0.001, 0, 0, 0, Double.valueOf(0)),
                new PIDConfig(0.1, 0, 0, 0, Double.valueOf(0)));
    }

    /**
     * Drives the robot using the drivetrain class
     * 
     * @param chassisSpeeds The target chassisSpeeds (Positive x away from drivers,
     *                      positive y to the left of drivers, ccw positive)
     * @param fieldRelative Whether the chassisSpeeds are field relative
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        driveTrain.drive(chassisSpeeds, fieldRelative);
    }

    /**
     * Drives the robot not field relative
     * 
     * @param chassisSpeeds The target chassisSpeeds
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.drive(chassisSpeeds, false);
    }

    /**
     * Sets the max translation speed of the robot
     * 
     * @param maxSpeed The max speed in m/s
     */
    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    /**
     * Gets the max translation speed of the robot
     * 
     * @return The max speed in m/s
     */
    public double getMaxSpeed() {
        return this.maxSpeed;
    }

    /**
     * Sets the max rotation speed of the robot
     * 
     * @param maxSpeed The max speed in rad/s
     */
    public void setMaxAngularSpeed(double maxAngularSpeed) {
        this.maxAngularSpeed = maxAngularSpeed;
    }

    /**
     * Gets the max rotation speed of the robot
     * 
     * @return The max speed in rad/s
     */
    public double getMaxAngularSpeed() {
        return this.maxAngularSpeed;
    }

    /**
     * Gets the current estimated pose2d on the field
     * 
     * @return The estimated pose2d
     */
    public Pose2d getPose2d() {
        return driveTrain.getPose2d();
    }

    /**
     * Gets the current estimated alliance relative pose2d
     * 
     * @return The estimated alliance relative pose2d
     */
    public Pose2d getAllianceRelativePose2d() {
        return driveTrain.getAllianceRelativePose2d();
    }

    /**
     * Resets the estimated pose2d
     * 
     * @param pose2d The pose to reset to
     */
    public void resetPose2d(Pose2d pose2d) {
        this.driveTrain.resetPose2d(pose2d);
    }

    /**
     * Resets the estimated alliance relative pose2d
     * 
     * @param pose2d The pose to reset to
     */
    public void resetAllianceRelativePose2d(Pose2d pose2d) {
        this.driveTrain.resetAllianceRelativePose2d(pose2d);
    }

    /**
     * Sets the gyro angle adjustment
     * 
     * @param angleAdjustment The gyro angle adjustment in degrees
     */
    public void setGyroAngleAdjustment(double angleAdjustment) {
        this.driveTrain.setGyroAngleAdjustment(angleAdjustment);
    }

    /**
     * Sets the target pose2d for translation and rotation pid loops
     * 
     * @param pose2d The target pose2d
     */
    public void setTargetPose2d(Pose2d pose2d) {
        this.driveTrain.setTargetPose2d(pose2d);
    }

    /**
     * Gets the current unwrapped gyro rotation
     * 
     * @return The current gyro rotation (ccw positive)
     */
    public Rotation2d getGyroRotation() {
        return this.driveTrain.getUnwrappedGyroRotation();
    }

    /**
     * Sets the gyro offset for the pose estimator
     * 
     * @param gyroOffset The gyro offset (ccw positive)
     */
    public void setPoseEstimatorGyroOffset(Rotation2d gyroOffset) {
        this.driveTrain.setPoseEstimatorGyroOffset(gyroOffset);
    }

    /**
     * Resets the gyro
     */
    public void resetGyro() {
        driveTrain.resetGyro();
    }

    /**
     * Sets the simulated gyro angle
     * 
     * @param simulatedAngle The simulated gyro angle in degrees (ccw positive)
     */
    public void setSimulatedGyroAngle(double simulatedAngle) {
        driveTrain.setSimulatedGyroAngle(simulatedAngle);
    }

    /**
     * Sets the estimated pose of the pose estimator
     */
    public void setPoseEstimatorPose2d(Pose2d pose2d) {
        driveTrain.setPoseEstimatorPose2d(pose2d);
    }
}
