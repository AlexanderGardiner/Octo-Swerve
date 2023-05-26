package frc.robot.Libraries.Swerve.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveKinematics {
    Translation2d[] modulePositions;
    public SwerveKinematics(Translation2d[] modulePositions) {
        this.modulePositions = modulePositions;
    }
    //TODO: need to implement multiple modules

    public SwerveModuleState[] calculateFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

        for (int i=0; i<modulePositions.length; i++) {
            Pose2d targetLocation = new Pose2d(chassisSpeeds.vxMetersPerSecond*0.02,
                                           chassisSpeeds.vyMetersPerSecond*0.02,
                                           new Rotation2d(chassisSpeeds.omegaRadiansPerSecond*0.02));
        
            Translation2d targetModulePosition = new Translation2d(modulePositions[i].getX() + targetLocation.getX(),
                                                                modulePositions[i].getY() + targetLocation.getY());  
                                                                
            targetModulePosition = new Translation2d(targetModulePosition.getX() * Math.cos(targetLocation.getRotation().getRadians())
                                                    - targetModulePosition.getY() * Math.sin(targetLocation.getRotation().getRadians()),
                                                    targetModulePosition.getY() * Math.cos(targetLocation.getRotation().getRadians())
                                                    - targetModulePosition.getX() * Math.sin(targetLocation.getRotation().getRadians()));

            double distanceToTargetModulePosition = Math.sqrt(Math.pow(targetModulePosition.getY(),2) + Math.pow(targetModulePosition.getX(), 2));
            double angle = (Math.atan2(targetModulePosition.getY(), targetModulePosition.getX()) - (Math.PI/4)) % (2 * Math.PI);
            moduleStates[i] = new SwerveModuleState(distanceToTargetModulePosition/0.02, new Rotation2d(angle));

        }
        
        
        return moduleStates;
        
    }
}
