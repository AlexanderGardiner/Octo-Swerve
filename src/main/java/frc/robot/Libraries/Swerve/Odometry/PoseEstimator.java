package frc.robot.Libraries.Swerve.Odometry;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseEstimator {
    Pose2d currentPose;
    ArrayList<SwerveModulePosition> previousModulePositions = new ArrayList<SwerveModulePosition>();
    Rotation2d gyroOffset = new Rotation2d();
    private static final double FIELD_WIDTH_METERS = 16.485;
    private static final double FIELD_HEIGHT_METERS = 8.02;

    /**
     * Creates a new pose estimator
     * 
     * @param initalPose The inital pose the robot starts at (Positive x away from
     *                   drivers, positive y to the left of drivers, ccw positive)
     */
    public PoseEstimator(Pose2d initalPose) {
        this.currentPose = initalPose;
        previousModulePositions.add(new SwerveModulePosition());
        previousModulePositions.add(new SwerveModulePosition());
        previousModulePositions.add(new SwerveModulePosition());
        previousModulePositions.add(new SwerveModulePosition());
    }

    /**
     * Updates the robot's pose to its new position
     * 
     * @param modulePositions The current positions of the swerve modules (In meters
     *                        and radians)
     * @param gyroAngle       The current robot gyro angle in radians (ccw positive)
     */
    public void updatePose(ArrayList<SwerveModulePosition> modulePositions, Rotation2d gyroAngle) {
        double sumXFromModules = 0;
        double sumYFromModules = 0;

        // Adding up all of the translations of the swerve modules
        for (int i = 0; i < modulePositions.size(); i++) {
            sumXFromModules += (modulePositions.get(i).distanceMeters - previousModulePositions.get(i).distanceMeters)
                    * Math.cos(modulePositions.get(i).angle.minus(gyroAngle).minus(gyroOffset).getRadians());

            sumYFromModules -= (modulePositions.get(i).distanceMeters - previousModulePositions.get(i).distanceMeters)
                    * Math.sin(modulePositions.get(i).angle.minus(gyroAngle).minus(gyroOffset).getRadians());
        }

        previousModulePositions = modulePositions;

        // Averaging the translations of the swerve modules
        double averageXFromModules = sumXFromModules / modulePositions.size();
        double averageYFromModules = sumYFromModules / modulePositions.size();

        // Applying average translations to the robot pose
        this.currentPose = new Pose2d(this.currentPose.getX()
                + averageXFromModules,
                this.currentPose.getY()
                        + averageYFromModules,
                gyroAngle.plus(gyroOffset));

    }

    /**
     * Gets the current robot pose
     * 
     * @return The current robot pose (Positive x away from drivers, positive y to
     *         the left of drivers, ccw positive)
     */
    public Pose2d getPose2d() {
        SmartDashboard.putString("Current Pose", currentPose.toString());
        SmartDashboard.putString("Pose estimator gyro offset", gyroOffset.toString());
        return this.currentPose;
    }

    /**
     * Gets the current robot pose, if the alliance is red then the 0,0 point shifts
     * to the top right corner when looking at a field2d widget
     * 
     * @return The alliance relative pose
     */
    public Pose2d getPose2dAllianceRelative() {
        Pose2d allianceRelativePose = currentPose;
        SmartDashboard.putString("Current Pose1", currentPose.toString() + "TEST");
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            allianceRelativePose = new Pose2d(FIELD_WIDTH_METERS - currentPose.getX(),
                    FIELD_HEIGHT_METERS - currentPose.getY(),
                    currentPose.getRotation().plus(new Rotation2d(Math.PI)));
        }
        SmartDashboard.putString("alliance relative pose2d", allianceRelativePose.toString() + "TEST!");
        return allianceRelativePose;
    }

    /**
     * Resets the current robot pose (Does not reset the gyro, uses an offset)
     * (Positive x away from drivers, positive y to the left of drivers, cw
     * positive)
     * 
     * @param pose2d The pose to reset to <b>(Angle is cw postitive)<b>
     */
    public void resetPose2d(Pose2d pose2d, ArrayList<SwerveModulePosition> modulePositions) {
        SmartDashboard.putString("Pose to reset to", pose2d.toString());
        this.gyroOffset = pose2d.getRotation().minus(currentPose.getRotation());
        this.currentPose = pose2d;
        this.previousModulePositions = modulePositions;
    }

    /**
     * Resets the current robot pose, if the alliance is red then the 0,0 point
     * shifts to the top right corner when looking at a field2d widget
     * 
     * @param pose2d          The current alliance relative robot pose
     * @param modulePositions The current swerve module positions
     */
    public void resetPose2dAllianceRelative(Pose2d pose2d, ArrayList<SwerveModulePosition> modulePositions) {
        Pose2d fieldRelativePose = pose2d;
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            fieldRelativePose = new Pose2d(FIELD_WIDTH_METERS - pose2d.getX(),
                    FIELD_HEIGHT_METERS - pose2d.getY(), pose2d.getRotation().minus(new Rotation2d(Math.PI)));
        }
        resetPose2d(fieldRelativePose, previousModulePositions);
    }

    /**
     * Sets the gyro offset that is used to avoid resetting the gyro when resetting
     * the pose
     * 
     * @param gyroOffset The offset to set (ccw positive)
     */
    public void setGyroOffset(Rotation2d gyroOffset) {
        this.gyroOffset = gyroOffset;
    }

    /**
     * Updates the current pose from an accurate pose measurement
     * 
     * @param poseMeasurement The pose to update the current pose with
     */
    public void addPoseMeasurement(Pose2d poseMeasurement) {
        this.currentPose = poseMeasurement;
    }

}
