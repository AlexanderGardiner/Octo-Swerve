package frc.robot.Libraries.Util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
    private AHRS navX;

    public Gyro() {
        this.navX = new AHRS(SPI.Port.kMXP);
    }

    public double getContinuousAngleDegrees() {
        return this.navX.getAngle();
    }

    public double getWrappedAngleDegrees() {
        return MathUtil.wrapToCircle(getContinuousAngleDegrees(), 360);
    }

    public Rotation2d getWrappedAngleRotation2D() {
        return new Rotation2d(Math.toRadians(getWrappedAngleDegrees()));
    }
}
