package frc.robot.Libraries.Swerve;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Libraries.Swerve.Math.SwerveKinematics;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.Gyro;
import frc.robot.Libraries.Util.PIDConfig;

public class Drivetrain {
    private ArrayList<SwerveModule> swerveModules = new ArrayList<SwerveModule>();
    private SwerveKinematics swerveDriveKinematics;
    private Gyro gyro;

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
     * (positive x driving right and positive y driving forward)
     */
    public Drivetrain(MotorType turnMotorTypes, MotorType driveMotorTypes, 
                      int[] turnMotorCanIDs, int[] driveMotorCanIDs,
                      PIDConfig[] turnMotorPIDConfigs, PIDConfig[] driveMotorPIDConfigs,
                      int turnMotorEncodersCountsPerRev, int driveMotorEncodersCountsPerRev,
                      int gearingTurnEncoderToOutput, int gearingDriveEncoderToOutput, int wheelRadius,
                      Translation2d[] modulePositions) {
        for (int i=0; i<4; i++) {
            swerveModules.add(new SwerveModule(turnMotorTypes,
                                               driveMotorTypes,
                                               turnMotorCanIDs[i],
                                               driveMotorCanIDs[i],
                                               turnMotorPIDConfigs[i],
                                               driveMotorPIDConfigs[i],
                                               turnMotorEncodersCountsPerRev,
                                               driveMotorEncodersCountsPerRev,
                                               gearingTurnEncoderToOutput,
                                               driveMotorEncodersCountsPerRev,
                                               wheelRadius));
        }

        swerveDriveKinematics = new SwerveKinematics(modulePositions);
        gyro = new Gyro();

    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, gyro.getWrappedAngleRotation2D());
        }
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.calculateFromChassisSpeeds(chassisSpeeds);

        for (int i=0; i<4; i++) {
            swerveModules.get(i).setTargetState(swerveModuleStates[i]);
        }
    }
}
