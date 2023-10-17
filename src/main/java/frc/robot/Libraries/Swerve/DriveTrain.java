package frc.robot.Libraries.Swerve;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Libraries.Swerve.Math.SwerveKinematics;
import frc.robot.Libraries.Swerve.Odometry.PoseEstimator;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.Gyro;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Libraries.Util.PIDConfig;

public class DriveTrain {
    private ArrayList<SwerveModule> swerveModules = new ArrayList<SwerveModule>();
    private SwerveKinematics swerveDriveKinematics;
    private Gyro gyro;
    private PoseEstimator poseEstimator;
    private boolean simulated;

    private Pose2d targetPose2d = new Pose2d();
    private PIDController translationPIDController;
    private PIDController rotationPidController;

    /**
     * Creates a swerve drivetrain object
     * 
     * @param turnMotorTypes                 The type of motor used to turn the
     *                                       modules
     * @param driveMotorTypes                The type of motor used to drive the
     *                                       modules
     * @param turnMotorCanIDs                The canIDs of the turn motors
     * @param driveMotorCanIDs               The canIDs of the drive motors
     * @param turnMotorPIDConfigs            The PIDConfigs of the turn motors
     * @param driveMotorPIDConfigs           The PIDConfigs of the drive motors
     * @param turnMotorEncodersCountsPerRev  The ticks per revolution of the turn
     *                                       encoders
     * @param driveMotorEncodersCountsPerRev The ticks per revolution of the drive
     *                                       encoders
     * @param gearingTurnEncoderToOutput     The gearing from the turn encoder to
     *                                       the output
     * @param gearingDriveEncoderToOutput    The gearing from the drive encoder to
     *                                       the output
     * @param wheelRadius                    The radius of the wheels
     * @param modulePositions                The positions of the modules relative
     *                                       to the center of the robot
     * @param turnMotorInverted              Whether the turn motors' directions are
     *                                       inverted
     * @param turnEncoderInverted            Whether the turn encoders are not in
     *                                       phase (inverted)
     * @param turnMotorInverted              Whether the drive motors' directions
     *                                       are inverted
     * @param turnEncoderInverted            Whether the drive encoders are not in
     *                                       phase (inverted)
     *                                       (positive x driving right and positive
     *                                       y driving forward)
     * @param simulated                      Whether the drivetrain is simulated
     * @param initalPose2d                   The inital pose of the robot on the
     *                                       field
     * @param translationPIDController       The pid controller for correcting error
     *                                       in translations of the robot
     * @param rotationPIDController          The pid controller for correcting error
     *                                       in rotations of the robot
     */
    public DriveTrain(MotorType turnMotorTypes, MotorType driveMotorTypes,
            int[] turnMotorCanIDs, int[] driveMotorCanIDs,
            PIDConfig[] turnMotorPIDConfigs, PIDConfig[] driveMotorPIDConfigs,
            int turnMotorEncodersCountsPerRev, int driveMotorEncodersCountsPerRev,
            double gearingTurnEncoderToOutput, double gearingDriveEncoderToOutput, double wheelRadius,
            Translation2d[] modulePositions,
            boolean[] turnMotorInverted, boolean[] turnEncoderInverted,
            boolean[] driveMotorInverted, boolean[] driveEncoderInverted,
            boolean simulated,
            Pose2d initalPose2d,
            PIDConfig translationPIDConfig, PIDConfig rotationPIDConfig,
            boolean loggingEnabled) {
        for (int i = 0; i < 4; i++) {
            swerveModules.add(new SwerveModule(turnMotorTypes, driveMotorTypes,
                    turnMotorCanIDs[i], driveMotorCanIDs[i],
                    turnMotorPIDConfigs[i], driveMotorPIDConfigs[i],
                    turnMotorEncodersCountsPerRev, driveMotorEncodersCountsPerRev,
                    gearingTurnEncoderToOutput, gearingDriveEncoderToOutput,
                    wheelRadius,
                    turnMotorInverted[i], turnEncoderInverted[i],
                    driveMotorInverted[i], driveEncoderInverted[i],
                    simulated));
            if (loggingEnabled) {
                SmartDashboard.putData("Swerve-Module-" + i, swerveModules.get(i));
            }
        }

        swerveDriveKinematics = new SwerveKinematics(modulePositions);
        gyro = new Gyro(simulated, 0);

        if (loggingEnabled) {
            SmartDashboard.putData("Gyro", gyro);
        }

        poseEstimator = new PoseEstimator(initalPose2d);
        targetPose2d = initalPose2d;
        this.simulated = simulated;
        this.translationPIDController = new PIDController(translationPIDConfig.getP(), translationPIDConfig.getI(),
                translationPIDConfig.getD());
        this.translationPIDController.setIntegratorRange(-translationPIDConfig.getIZone(),
                translationPIDConfig.getIZone());
        this.rotationPidController = new PIDController(rotationPIDConfig.getP(), rotationPIDConfig.getI(),
                rotationPIDConfig.getD());
        this.rotationPidController.setIntegratorRange(-rotationPIDConfig.getIZone(),
                rotationPIDConfig.getIZone());
        this.rotationPidController.enableContinuousInput(-Math.PI, Math.PI);

    }

    /**
     * Drives the robot and updates the robot's pose
     * 
     * @param chassisSpeeds The target speed of the robot (Positive x away from
     *                      drivers, positive y to the left of drivers, ccw
     *                      positive)
     * @param fieldRelative Whether the movement is field relative
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative) {
            // Calculates the target position and feeds it into a pid loop to ensure
            // accurate driving then calculates the robot relative speeds
            Pose2d currentPose = poseEstimator.getPose2d();

            // Sets the x and/or y of the target position to the current x/y if there is no
            // input along that axis
            double targetPoseX = targetPose2d.getX();
            double targetPoseY = targetPose2d.getY();
            Rotation2d targetPoseRotation = targetPose2d.getRotation();
            if (chassisSpeeds.vxMetersPerSecond == 0) {
                targetPoseX = currentPose.getX();
            }

            if (chassisSpeeds.vyMetersPerSecond == 0) {
                targetPoseY = currentPose.getY();
            }

            targetPose2d = new Pose2d(targetPoseX, targetPoseY, targetPoseRotation);

            // Calculates the target position with the chassis speeds added and calculates
            // speeds
            targetPose2d = new Pose2d(targetPose2d.getX() + (chassisSpeeds.vxMetersPerSecond * 0.02),
                    targetPose2d.getY() + (chassisSpeeds.vyMetersPerSecond * 0.02),
                    new Rotation2d(
                            targetPose2d.getRotation().getRadians() + (chassisSpeeds.omegaRadiansPerSecond * 0.02)));

            // SmartDashboard.putString("targetPose", targetPose2d.toString());
            // SmartDashboard.putString("currentPose", currentPose.toString());
            chassisSpeeds = new ChassisSpeeds(
                    chassisSpeeds.vxMetersPerSecond
                            + (translationPIDController.calculate(currentPose.getX(),
                                    targetPose2d.getX())
                                    /
                                    0.02),
                    chassisSpeeds.vyMetersPerSecond
                            + (translationPIDController.calculate(currentPose.getY(),
                                    targetPose2d.getY())
                                    /
                                    0.02),
                    chassisSpeeds.omegaRadiansPerSecond
                            + (rotationPidController.calculate(currentPose.getRotation().getRadians(),
                                    targetPose2d.getRotation().getRadians()) / 0.02));

            if (MathUtil.isWithinTolerance(chassisSpeeds.vxMetersPerSecond, 0, 0.01)) {
                chassisSpeeds.vxMetersPerSecond = 0;
            }

            if (MathUtil.isWithinTolerance(chassisSpeeds.vyMetersPerSecond, 0, 0.01)) {
                chassisSpeeds.vyMetersPerSecond = 0;
            }

            if (MathUtil.isWithinTolerance(chassisSpeeds.omegaRadiansPerSecond, 0, 0.01)) {
                chassisSpeeds.omegaRadiansPerSecond = 0;
            }
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getWrappedAngleRotation2D());
        }

        // Sets the swerve modules to the target states
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.calculateFromChassisSpeeds(chassisSpeeds);

        for (int i = 0; i < 4; i++) {
            swerveModules.get(i).setTargetState(swerveModuleStates[i]);
        }

        updatePose();

        if (simulated) {
            gyro.setSimulatedRotationSpeed(Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
        }
    }

    /**
     * Updates current pose from current module states
     */
    public void updatePose() {
        ArrayList<SwerveModulePosition> modulePositions = new ArrayList<SwerveModulePosition>();

        for (int i = 0; i < 4; i++) {
            modulePositions.add(swerveModules.get(i).getModulePosition());
        }

        poseEstimator.updatePose(modulePositions,
                gyro.getRotation2d());
    }

    /**
     * Gets the robot estimated pose on the field
     * 
     * @return The robot's pose
     */
    public Pose2d getPose2d() {
        return poseEstimator.getPose2d();
    }

    /**
     * Gets the current robot pose, if the alliance is red then the 0,0 point shifts
     * to the top right corner when looking at a field2d widget
     * 
     * @return The current robot pose
     */
    public Pose2d getAllianceRelativePose2d() {
        return poseEstimator.getPose2dAllianceRelative();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    /**
     * Gets the unwrapped rotation
     * 
     * @return The unwrapped rotation of the gyro
     */
    public Rotation2d getUnwrappedGyroRotation() {
        return this.gyro.getWrappedAngleRotation2D();
    }

    /**
     * Resets the robot pose
     * 
     * @param pose2d The pose to reset to
     */
    public void resetPose2d(Pose2d pose2d) {
        ArrayList<SwerveModulePosition> modulePositions = new ArrayList<SwerveModulePosition>();

        for (int i = 0; i < 4; i++) {
            modulePositions.add(swerveModules.get(i).getModulePosition());
        }

        this.targetPose2d = pose2d;

        poseEstimator.resetPose2d(pose2d, modulePositions);
    }

    /**
     * Resets the robot pose, if the alliance is red then the 0,0 point
     * shifts to the top right corner when looking at a field2d widget
     * 
     * @param pose2d The current robot pose
     */
    public void resetAllianceRelativePose2d(Pose2d pose2d) {
        ArrayList<SwerveModulePosition> modulePositions = new ArrayList<SwerveModulePosition>();

        for (int i = 0; i < 4; i++) {
            modulePositions.add(swerveModules.get(i).getModulePosition());
        }

        this.targetPose2d = pose2d;

        poseEstimator.resetPose2dAllianceRelative(pose2d, modulePositions);
    }

    /**
     * Offset the gyro
     * 
     * @param gyroOffset The amount to offset the gyro by in degrees (ccw positive)
     */
    public void setGyroAngleAdjustment(double angleAdjustment) {
        gyro.setAngleAdjustment(angleAdjustment);
    }

    /**
     * Sets the target pose2d of the drivetrain
     * 
     * @param pose2d The target pose of the drivetrain (Positive x away from
     *               drivers, positive y to the left of drivers, ccw positive)
     */
    public void setTargetPose2d(Pose2d pose2d) {
        this.targetPose2d = pose2d;
    }

    /**
     * Gets the target pose2d of the drivetrain
     */
    public Pose2d getTargetPose2d() {
        return this.targetPose2d;
    }

    /**
     * Sets the gyro offset that the pose estimator uses to avoid resetting the gyro
     * 
     * @param gyroOffset The offset to set (ccw positive)
     */
    public void setPoseEstimatorGyroOffset(Rotation2d gyroOffset) {
        this.poseEstimator.setGyroOffset(gyroOffset);
    }

    /**
     * Resets the robot's gyro
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Sets the simulated angle of the gyro (degrees ccw positive)
     * 
     * @param simulatedAngle
     */
    public void setSimulatedGyroAngle(double simulatedAngle) {
        gyro.setSimulatedAngle(simulatedAngle);
    }

    public void setPoseEstimatorPose2d(Pose2d pose2d) {
        poseEstimator.addPoseMeasurement(pose2d);
    }
}
