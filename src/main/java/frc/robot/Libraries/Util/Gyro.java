package frc.robot.Libraries.Util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private AHRS navX;
    private boolean simulated;
    private double simulatedAngleDegrees;
    private double simulatedAngleOffset = 0;
    private double simulatedRotationSpeed = 0; // Radians per second
    private double lastTimeSimulatedRotationUpdated = 0;

    /** Creates a gyro wrapper class for a navX connected to a robotrio
     * @param simulated Whether the gyro is simulated
     * @param initialAngle The starting angle the robot starts at
     */
    public Gyro(boolean simulated, double initialAngle) {
        this.navX = new AHRS(SPI.Port.kMXP);
        this.simulated = simulated;
        if (simulated) {
            this.simulatedAngleDegrees = initialAngle;
        }
        
    }

    /** Gets the continuous angle of the gyro
     * @return The continuous angle of the gyro in degrees (It goes beyond 1 full rotation)
     */
    public double getContinuousAngleDegrees() {
        if (simulated) {
            simulatedAngleDegrees += simulatedRotationSpeed * (Timer.getFPGATimestamp()-lastTimeSimulatedRotationUpdated);
            lastTimeSimulatedRotationUpdated = Timer.getFPGATimestamp();
            return simulatedAngleDegrees + simulatedAngleOffset;
        } else {
            return this.navX.getAngle();
        }    
    }

    /** Gets the wrapped angle of the gyro
     * @return The wrapped angle of the gyro in degrees (It is limited between 0 and 360)
     */
    public double getWrappedAngleDegrees() {
        return MathUtil.wrapToCircle(getContinuousAngleDegrees(), 360);
    }

    /** Gets the unwrapped angle of the gyro as a rotation 2D
     * @return The unwrapped angle as a rotation 2D
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(getContinuousAngleDegrees()));
        
    }

    /** Gets the wrapped angle of the gyro
     * @return The wrapped angle as a rotation 2D
     */
    public Rotation2d getWrappedAngleRotation2D() {
        SmartDashboard.putNumber("wrapped angle", getWrappedAngleDegrees());
        return new Rotation2d(Math.toRadians(getWrappedAngleDegrees()));
        
    }

    public void setAngleOffset(double angleOffset) {
        if (simulated) {
            simulatedAngleOffset = angleOffset;
        } else {
            navX.setAngleAdjustment(angleOffset);
        }
        
    }

    public void setSimulatedRotationSpeed(double rotationSpeed) {
        simulatedAngleDegrees += simulatedRotationSpeed * (Timer.getFPGATimestamp()-lastTimeSimulatedRotationUpdated);
        lastTimeSimulatedRotationUpdated = Timer.getFPGATimestamp();
        this.simulatedRotationSpeed = rotationSpeed;
    }
}
