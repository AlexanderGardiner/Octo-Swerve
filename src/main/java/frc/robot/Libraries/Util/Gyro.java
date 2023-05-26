package frc.robot.Libraries.Util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
    private AHRS navX;

    /** Creates a gyro wrapper class for a navX connected to a robotrio
     */
    public Gyro() {
        this.navX = new AHRS(SPI.Port.kMXP);
    }

    /** Gets the continuous angle of the gyro
     * @return The continuous angle of the gyro in degrees (It goes beyond 1 full rotation)
     */
    public double getContinuousAngleDegrees() {
        return this.navX.getAngle();
    }

    /** Gets the wrapped angle of the gyro
     * @return The wrapped angle of the gyro in degrees (It is limited between 0 and 360)
     */
    public double getWrappedAngleDegrees() {
        return MathUtil.wrapToCircle(getContinuousAngleDegrees(), 360);
    }

    /** Gets the wrapped angle of the gyro
     * @return The wrapped angle as a rotation 2D
     */
    public Rotation2d getWrappedAngleRotation2D() {
        return new Rotation2d(Math.toRadians(getWrappedAngleDegrees()));
    }
}
