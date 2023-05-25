package frc.robot.Libraries.Swerve;

import java.util.ArrayList;

import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Util.PIDConfig;

public class Drivetrain {
    private ArrayList<SwerveModule> swerveModules = new ArrayList<SwerveModule>();

    public Drivetrain(MotorType turnMotorTypes, MotorType driveMotorTypes, 
                      int[] turnMotorCanIDs, int[] driveMotorCanIDs,
                      PIDConfig[] turnMotorPIDConfigs, PIDConfig[] driveMotorPIDConfigs,
                      int turnMotorEncodersCountsPerRev, int driveMotorEncodersCountsPerRev,
                      int gearingTurnEncoderToOutput, int gearingDriveEncoderToOutput, int wheelRadius) {
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


    }
}
