package frc.robot.Libraries.Swerve.Odometry;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator {
    Pose2d currentPose;
    ArrayList<SwerveModulePosition> previousModulePositions = new ArrayList<SwerveModulePosition>();

    /** Creates a new pose estimator
     * @param initalPose The inital pose the robot starts at
     */
    public PoseEstimator(Pose2d initalPose) {
        this.currentPose = initalPose;
        previousModulePositions.add(new SwerveModulePosition());
        previousModulePositions.add(new SwerveModulePosition());
        previousModulePositions.add(new SwerveModulePosition());
        previousModulePositions.add(new SwerveModulePosition());
    }

    /** Updates the robot's pose to its new position
     * @param modulePositions The current positions of the swerve modules (In meters and radians)
     * @param gyroAngle The current robot gyro angle in radians
     */
    public void updatePose(ArrayList<SwerveModulePosition> modulePositions, Rotation2d gyroAngle) {
        double sumXFromModules = 0;
        double sumYFromModules = 0;

        for (int i=0; i<modulePositions.size(); i++) {
            sumXFromModules += (modulePositions.get(i).distanceMeters-previousModulePositions.get(i).distanceMeters) 
                                * Math.sin(modulePositions.get(i).angle.plus(gyroAngle).getRadians());

            sumYFromModules += (modulePositions.get(i).distanceMeters-previousModulePositions.get(i).distanceMeters) 
                                * Math.cos(modulePositions.get(i).angle.plus(gyroAngle).getRadians());
        }

        double averageXFromModules = sumXFromModules / modulePositions.size();
        double averageYFromModules = sumYFromModules / modulePositions.size();

        this.currentPose = new Pose2d(this.currentPose.getX() 
                                      + averageXFromModules,
                                      this.currentPose.getY() 
                                      + averageYFromModules,
                                      gyroAngle);

        previousModulePositions = modulePositions;
    }

    /**
     * @return The current robot pose
     */
    public Pose2d getPose2d() {
        return this.currentPose;
    }
}
