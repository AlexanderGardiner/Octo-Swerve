package frc.robot.Libraries.Swerve.Odometry;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator {
    Pose2d currentPose;
    ArrayList<SwerveModulePosition> previousModulePositions = new ArrayList<SwerveModulePosition>();
    Rotation2d gyroOffset = new Rotation2d();

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
        return this.currentPose;
    }

    /**
     * Resets the current robot pose (Does not reset the gyro, uses an offset)
     * (Positive x away from drivers, positive y to the left of drivers, ccw
     * positive)
     * 
     * @param pose2d The pose to reset to <b>(Angle is cw postitive)<b>
     */
    public void resetPose2d(Pose2d pose2d, ArrayList<SwerveModulePosition> modulePositions) {
        this.gyroOffset = pose2d.getRotation().minus(currentPose.getRotation());
        this.currentPose = pose2d;
        this.previousModulePositions = modulePositions;
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
