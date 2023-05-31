package frc.robot.Libraries.Swerve;

import java.util.ArrayList;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Libraries.Swerve.Math.SwerveKinematics;
import frc.robot.Libraries.Swerve.Odometry.PoseEstimator;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.Gyro;
import frc.robot.Libraries.Util.PIDConfig;

public class DriveTrain {
    private ArrayList<SwerveModule> swerveModules = new ArrayList<SwerveModule>();
    private SwerveKinematics swerveDriveKinematics;
    private Gyro gyro;
    private PoseEstimator poseEstimator;
    private boolean simulated;
    private double simulatedRotation = 0; // Radians
    private double simulatedRotationSpeed = 0; // Radians per second
    private double lastTimeSimulatedRotationUpdated = 0;

    /** Creates a swerve drivetrain object
     * @param turnMotorTypes The type of motor used to turn the modules
     * @param driveMotorTypes The type of motor used to drive the modules
     * @param turnMotorCanIDs The canIDs of the turn motors
     * @param driveMotorCanIDs The canIDs of the drive motors
     * @param turnMotorPIDConfigs The PIDConfigs of the turn motors
     * @param driveMotorPIDConfigs The PIDConfigs of the drive motors
     * @param turnMotorEncodersCountsPerRev The ticks per revolution of the turn encoders
     * @param driveMotorEncodersCountsPerRev The ticks per revolution of the drive encoders
     * @param gearingTurnEncoderToOutput The gearing from the turn encoder to the output
     * @param gearingDriveEncoderToOutput The gearing from the drive encoder to the output
     * @param wheelRadius The radius of the wheels
     * @param modulePositions The positions of the modules relative to the center of the robot 
     * @param turnMotorInverted Whether the turn motors' directions are inverted
     * @param turnEncoderInverted Whether the turn encoders are not in phase (inverted)
     * @param turnMotorInverted Whether the drive motors' directions are inverted
     * @param turnEncoderInverted Whether the drive encoders are not in phase (inverted)
     * (positive x driving right and positive y driving forward)
     * @param simulated Whether the drivetrain is simulated
     */
    public DriveTrain(MotorType turnMotorTypes, MotorType driveMotorTypes, 
                      int[] turnMotorCanIDs, int[] driveMotorCanIDs,
                      PIDConfig[] turnMotorPIDConfigs, PIDConfig[] driveMotorPIDConfigs,
                      int turnMotorEncodersCountsPerRev, int driveMotorEncodersCountsPerRev,
                      double gearingTurnEncoderToOutput, double gearingDriveEncoderToOutput, double wheelRadius,
                      Translation2d[] modulePositions,
                      boolean[] turnMotorInverted, boolean[] turnEncoderInverted,
                      boolean[] driveMotorInverted, boolean[] driveEncoderInverted,
                      boolean simulated) {
        for (int i=0; i<4; i++) {
            swerveModules.add(new SwerveModule(turnMotorTypes, driveMotorTypes,
                                               turnMotorCanIDs[i], driveMotorCanIDs[i],
                                               turnMotorPIDConfigs[i], driveMotorPIDConfigs[i],
                                               turnMotorEncodersCountsPerRev, driveMotorEncodersCountsPerRev,
                                               gearingTurnEncoderToOutput, gearingDriveEncoderToOutput,
                                               wheelRadius,
                                               turnMotorInverted[i], turnEncoderInverted[i],
                                               driveMotorInverted[i], driveEncoderInverted[i],
                                               simulated));
        }

        swerveDriveKinematics = new SwerveKinematics(modulePositions);
        gyro = new Gyro();
        poseEstimator = new PoseEstimator(new Pose2d());
        this.simulated = simulated;

    }

    /** Drives the robot and updates the robot's pose
     * @param chassisSpeeds The target speed of the robot (x, y, and theta)
     * @param fieldRelative Whether the movement is field relative
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative) {
            if (simulated) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, new Rotation2d(simulatedRotation));
            } else {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getWrappedAngleRotation2D());
            }
            
        }
        
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.calculateFromChassisSpeeds(chassisSpeeds);

        for (int i=0; i<4; i++) {
            swerveModules.get(i).setTargetState(swerveModuleStates[i]);
        }

        ArrayList<SwerveModulePosition> modulePositions = new ArrayList<SwerveModulePosition>();

        for (int i=0; i<4; i++) {
            modulePositions.add(swerveModules.get(i).getModulePosition());
        }

        if (simulated) {
            simulatedRotation += simulatedRotationSpeed * (Timer.getFPGATimestamp()-lastTimeSimulatedRotationUpdated);
            poseEstimator.updatePose(modulePositions, 
                                 new Rotation2d(simulatedRotation));
            simulatedRotationSpeed = chassisSpeeds.omegaRadiansPerSecond;
            lastTimeSimulatedRotationUpdated = Timer.getFPGATimestamp();
        } else {
            poseEstimator.updatePose(modulePositions, 
                                 gyro.getWrappedAngleRotation2D());
        }
        

    }

    /** Gets the robot estimated pose on the field
     * @return The robot's pose
     */
    public Pose2d getPose2d() {
        return poseEstimator.getPose2d();
    }

    /** Resets the robot pose
     * @param pose2d The pose to reset to
     */
    public void resetPose2d(Pose2d pose2d) {
        ArrayList<SwerveModulePosition> modulePositions = new ArrayList<SwerveModulePosition>();

        for (int i=0; i<4; i++) {
            modulePositions.add(swerveModules.get(i).getModulePosition());
        }

        poseEstimator.resetPose2d(pose2d, modulePositions);
    } 
}
