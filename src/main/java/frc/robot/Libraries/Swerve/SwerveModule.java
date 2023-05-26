package frc.robot.Libraries.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Libraries.Swerve.Util.DriveMotor;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Swerve.Util.StateOptimizer;
import frc.robot.Libraries.Swerve.Util.TurnMotor;
import frc.robot.Libraries.Util.PIDConfig;

public class SwerveModule {
    TurnMotor turnMotor;
    DriveMotor driveMotor;

    private double turnEncoderTickToRadians;
    private double driveEncoderTickToMeters;

    // TODO: Need to add option for inverted motor/encoder
    //TODO: Need to add position of module in constructor
    public SwerveModule(MotorType turnMotorType, MotorType driveMotorType, 
                  int turnMotorCanID, int driveMotorCanID,
                  PIDConfig turnMotorPIDConfig, PIDConfig driveMotorPIDConfig,
                  int turnMotorEncoderCountsPerRev, int driveMotorEncoderCountsPerRev,
                  int gearingTurnEncoderToOutput, int gearingDriveEncoderToOutput, int wheelRadius) {
        turnMotor = new TurnMotor(turnMotorType, turnMotorCanID, turnMotorPIDConfig, driveMotorEncoderCountsPerRev);
        driveMotor = new DriveMotor(driveMotorType, driveMotorCanID, driveMotorPIDConfig, driveMotorEncoderCountsPerRev);

        turnEncoderTickToRadians = (2 * Math.PI * (1/gearingTurnEncoderToOutput)) / turnMotorEncoderCountsPerRev;
        driveEncoderTickToMeters = (2 * Math.PI * wheelRadius * (1/gearingDriveEncoderToOutput)) / driveMotorEncoderCountsPerRev;
    }

    public void setTargetVelocityMeters(double velocity) {
        driveMotor.setTargetVelocityTicks(velocity);
    }

    public void setTargetAngleRadians(double angle) {
        driveMotor.setTargetVelocityTicks(angle);
    }

    public double getDistanceMeters() {
        return driveMotor.getEncoderPositionTicks() * driveEncoderTickToMeters;
    }

    public double getVelocityMeters() {
        return driveMotor.getEncoderVelocityTicks() * driveEncoderTickToMeters;
    }

    public double getAngleRadians() {
        return turnMotor.getEncoderPositionTicks() * turnEncoderTickToRadians;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistanceMeters(), new Rotation2d(getAngleRadians()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMeters(), new Rotation2d(getAngleRadians()));
    }

    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = StateOptimizer.optimizeSwerveStates(state, getAngleRadians());
        setTargetVelocityMeters(optimizedState.speedMetersPerSecond);
        setTargetAngleRadians(optimizedState.angle.getRadians());
    }

}
