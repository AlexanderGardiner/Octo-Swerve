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
                        double gearingTurnEncoderToOutput, double gearingDriveEncoderToOutput, int wheelRadius) {
        turnMotor = new TurnMotor(turnMotorType, turnMotorCanID, turnMotorPIDConfig, driveMotorEncoderCountsPerRev);
        driveMotor = new DriveMotor(driveMotorType, driveMotorCanID, driveMotorPIDConfig, driveMotorEncoderCountsPerRev);

        turnEncoderTickToRadians = (2.0 * Math.PI * (1.0/gearingTurnEncoderToOutput)) / turnMotorEncoderCountsPerRev;
        driveEncoderTickToMeters = (2.0 * Math.PI * wheelRadius * (1.0/gearingDriveEncoderToOutput)) / driveMotorEncoderCountsPerRev;
    }

    /** Sets the  target velocity of the module
     * @param velocity The target velocity in ticks per 100ms;
     */
    public void setTargetVelocityMeters(double velocity) {
        driveMotor.setTargetVelocityTicks(velocity);
    }

    /** Sets the  target angle of the module
     * @param angle The target angle in radians;
     */
    public void setTargetAngleRadians(double angle) {
        turnMotor.setTargetPositionTicks(angle/turnEncoderTickToRadians);
    }

    /** Gets the distance travelled by the drive motor
     * @return The distance travelled by the drive motor in meters
     */
    public double getDistanceMeters() {
        return driveMotor.getEncoderPositionTicks() * driveEncoderTickToMeters;
    }

    /** Gets the velocity of the drive motor
     * @return The velocity of the drive motor in m/s
     */
    public double getVelocityMeters() {
        return driveMotor.getEncoderVelocityTicks() * driveEncoderTickToMeters * 10;
    }

    /** Gets the angle of the turn motor in radians
     * @return The angle of the turn motor in radians
     */
    public double getAngleRadians() {
        return turnMotor.getEncoderPositionTicks() * turnEncoderTickToRadians;
    }

    /** Gets the module's position
     * @return The swerve module position (in meters and radians)
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistanceMeters(), new Rotation2d(getAngleRadians()));
    }

    /** Gets the state of the module
     * @return The swerve module state (in m/s and radians)
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMeters(), new Rotation2d(getAngleRadians()));
    }

    /** Sets the target state of the module
     * @param state The target state (in m/s and radians)
     */
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = StateOptimizer.optimizeSwerveStates(state, getAngleRadians());
        setTargetVelocityMeters(optimizedState.speedMetersPerSecond);
        setTargetAngleRadians(optimizedState.angle.getRadians());
    }

}
