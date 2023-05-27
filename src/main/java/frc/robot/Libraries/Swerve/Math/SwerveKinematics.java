package frc.robot.Libraries.Swerve.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveKinematics {
    Translation2d[] modulePositions;

    /** Creates a new SwerveKinematics
     * @param modulePositions The positions of the modules relative to the center of the robot
     */
    public SwerveKinematics(Translation2d[] modulePositions) {
        this.modulePositions = modulePositions;
    }

    /** Calculates the swerve module states to reach a target chassisSpeeds
     * @param chassisSpeeds The target chassisSpeeds
     * @return The calculated swerve module states
     */
    public SwerveModuleState[] calculateFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

        for (int i=0; i<modulePositions.length; i++) {
            Pose2d targetLocation = new Pose2d(chassisSpeeds.vxMetersPerSecond*0.02,
                                           chassisSpeeds.vyMetersPerSecond*0.02,
                                           new Rotation2d(chassisSpeeds.omegaRadiansPerSecond*0.02));

            Translation2d targetModulePosition = new Translation2d(modulePositions[i].getX() * Math.cos(targetLocation.getRotation().getRadians())
                                                                   + modulePositions[i].getY() * Math.sin(targetLocation.getRotation().getRadians()),
                                                                   modulePositions[i].getY() * Math.cos(targetLocation.getRotation().getRadians())
                                                                  - modulePositions[i].getX() * Math.sin(targetLocation.getRotation().getRadians()));

            targetModulePosition = new Translation2d(targetModulePosition.getX() + targetLocation.getX(),
                                                     targetModulePosition.getY() + targetLocation.getY());  
                                                                
            

            double distanceToTargetModulePosition = Math.sqrt(Math.pow((targetModulePosition.getY()-modulePositions[i].getY()),2) + Math.pow((targetModulePosition.getX()-modulePositions[i].getX()), 2));
            double angle = (Math.atan2(targetModulePosition.getX()-modulePositions[i].getX(), targetModulePosition.getY()-modulePositions[i].getY()));// - (Math.PI/4)) % (2 * Math.PI);
            moduleStates[i] = new SwerveModuleState(distanceToTargetModulePosition/0.02, new Rotation2d(angle));

        }
        
        
        return moduleStates;
        
    }
}
