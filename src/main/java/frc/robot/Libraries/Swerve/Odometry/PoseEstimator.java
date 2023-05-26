package frc.robot.Libraries.Swerve.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator {
    Pose2d currentPose;
    SwerveModulePosition[] previousModulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    public PoseEstimator(Pose2d initalPose) {
        this.currentPose = initalPose;
    }

    public void updatePose(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
        double sumXFromModules = 0;
        double sumYFromModules = 0;

        for (int i=0; i<modulePositions.length; i++) {
            sumXFromModules += (modulePositions[i].distanceMeters-previousModulePositions[i].distanceMeters) 
                                * Math.sin(modulePositions[i].angle.plus(gyroAngle).getRadians());

            sumYFromModules += (modulePositions[i].distanceMeters-previousModulePositions[i].distanceMeters) 
                                * Math.cos(modulePositions[i].angle.plus(gyroAngle).getRadians());
        }

        double averageXFromModules = sumXFromModules / modulePositions.length;
        double averageYFromModules = sumYFromModules / modulePositions.length;

        this.currentPose = new Pose2d(this.currentPose.getX() 
                                      + averageXFromModules,
                                      this.currentPose.getY() 
                                      + averageYFromModules,
                                      gyroAngle);

        previousModulePositions = modulePositions;
    }

    public Pose2d getPose2d() {
        return this.currentPose;
    }
}
