package frc.robot.Libraries.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Libraries.Swerve.Util.DriveMotor;
import frc.robot.Libraries.Swerve.Util.MotorType;
import frc.robot.Libraries.Swerve.Util.StateOptimizer;
import frc.robot.Libraries.Swerve.Util.TurnMotor;
import frc.robot.Libraries.Util.PIDConfig;

public class SwerveModule implements Sendable {
    TurnMotor turnMotor;
    DriveMotor driveMotor;

    private double turnEncoderTickToRadians;
    private double driveEncoderTickToMeters;

    /**
     * Creates a swerve module
     * 
     * @param turnMotorType                 The type of the turn motor
     * @param driveMotorType                The type of the drive motor
     * @param turnMotorCanID                The canID of the turn motor
     * @param driveMotorCanID               The canID of the drive motor
     * @param turnMotorPIDConfig            The turn motor PID config
     * @param driveMotorPIDConfig           The drive motor PID config
     * @param turnMotorEncoderCountsPerRev  The encoder ticks per revolution for the
     *                                      turn motor
     * @param driveMotorEncoderCountsPerRev The encoder ticks per revolution for the
     *                                      drive motor
     * @param gearingTurnEncoderToOutput    The gearing from the turn encoder to the
     *                                      output
     * @param gearingDriveEncoderToOutput   The gearing from the drive encoder to
     *                                      the output
     * @param wheelRadius                   The wheel radius of the module
     * @param turnMotorInverted             Whether the turn motor is inverted
     * @param turnEncoderInverted           Whether the turn encoder is in phase
     *                                      (inverted)
     * @param driveMotorInverted            Whether the drive motor is inverted
     * @param driveEncoderInverted          Whether the drive encoder is in phase
     *                                      (inverted)
     * @param simulated                     Whether the module is simulated
     */
    public SwerveModule(MotorType turnMotorType, MotorType driveMotorType,
            int turnMotorCanID, int driveMotorCanID,
            PIDConfig turnMotorPIDConfig, PIDConfig driveMotorPIDConfig,
            int turnMotorEncoderCountsPerRev, int driveMotorEncoderCountsPerRev,
            double gearingTurnEncoderToOutput, double gearingDriveEncoderToOutput,
            double wheelRadius,
            boolean turnMotorInverted, boolean turnEncoderInverted,
            boolean driveMotorInverted, boolean driveEncoderInverted,
            boolean simulated) {
        turnMotor = new TurnMotor(turnMotorType, turnMotorCanID, turnMotorPIDConfig, driveMotorEncoderCountsPerRev,
                turnMotorInverted, turnEncoderInverted, simulated);
        driveMotor = new DriveMotor(driveMotorType, driveMotorCanID, driveMotorPIDConfig, driveMotorEncoderCountsPerRev,
                driveMotorInverted, driveEncoderInverted, simulated);

        turnEncoderTickToRadians = (2.0 * Math.PI * (1.0 / gearingTurnEncoderToOutput)) / turnMotorEncoderCountsPerRev;
        driveEncoderTickToMeters = (2.0 * Math.PI * wheelRadius * (1.0 / gearingDriveEncoderToOutput))
                / driveMotorEncoderCountsPerRev;
    }

    /**
     * Sets the target velocity of the module
     * 
     * @param velocity The target velocity in m/s;
     */
    public void setTargetVelocityMeters(double velocity) {
        driveMotor.setTargetVelocityTicks((velocity / driveEncoderTickToMeters) / 10.0);
    }

    /**
     * Sets the target angle of the module
     * 
     * @param angle The target angle in radians;
     */
    public void setTargetAngleRadians(double angle) {
        turnMotor.setTargetPositionTicks(angle / turnEncoderTickToRadians);
    }

    /**
     * Gets the distance travelled by the drive motor
     * 
     * @return The distance travelled by the drive motor in meters
     */
    public double getDistanceMeters() {
        return driveMotor.getEncoderPositionTicks() * driveEncoderTickToMeters;
    }

    /**
     * Gets the velocity of the drive motor
     * 
     * @return The velocity of the drive motor in m/s
     */
    public double getVelocityMeters() {
        return driveMotor.getEncoderVelocityTicks() * driveEncoderTickToMeters * 10;
    }

    /**
     * Gets the angle of the turn motor in radians
     * 
     * @return The angle of the turn motor in radians
     */
    public double getAngleRadians() {
        return turnMotor.getEncoderPositionTicks() * turnEncoderTickToRadians;
    }

    /**
     * Gets the module's position
     * 
     * @return The swerve module position (in meters and radians)
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistanceMeters(), new Rotation2d(getAngleRadians()));
    }

    /**
     * Gets the state of the module
     * 
     * @return The swerve module state (in m/s and radians)
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMeters(), new Rotation2d(getAngleRadians()));
    }

    /**
     * Sets the target state of the module
     * 
     * @param state The target state (in m/s and radians)
     */
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = StateOptimizer.optimizeSwerveStates(state, getAngleRadians());
        setTargetVelocityMeters(optimizedState.speedMetersPerSecond);
        setTargetAngleRadians(optimizedState.angle.getRadians());
    }

    /**
     * Sets the sendable properties
     * 
     * @param builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Drive-Encoder-Distance-(m)", () -> getDistanceMeters(), null);
        builder.addDoubleProperty("Drive-Encoder-Velocity-(m/s)", () -> getVelocityMeters(), null);
        builder.addDoubleProperty("Turn-Encoder-Angle-(radians)", () -> getAngleRadians(), null);

    }

}
