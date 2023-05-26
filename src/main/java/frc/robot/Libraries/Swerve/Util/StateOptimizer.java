package frc.robot.Libraries.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Libraries.Util.MathUtil;

public class StateOptimizer {
    public static double getAngleDiff(double src, double target) {
        double diff = target - src;
        if (Math.abs(diff) <= Math.PI) {
            return diff;
        }

        if (diff > 0) {
            diff -= 2 * Math.PI;
        } else {
            diff += 2 * Math.PI;
        }

        return diff;
    }

    public static double getClosestAngle(double clampedAngle, double target) {
        double diff;
        double positiveTarget = target + Math.PI;
        double negativeTarget = target - Math.PI;
        if (Math.abs(getAngleDiff(clampedAngle, positiveTarget)) <= Math
                .abs(getAngleDiff(clampedAngle, negativeTarget))) {
            diff = getAngleDiff(clampedAngle, positiveTarget);
        } else {
            diff = getAngleDiff(clampedAngle, negativeTarget);
        }

        if (Math.abs(getAngleDiff(clampedAngle, target)) <= Math.abs(diff)) {
            diff = getAngleDiff(clampedAngle, target);
        }

        return diff;
    }

    public static SwerveModuleState optimizeSwerveStates(SwerveModuleState state, double moduleAngle) {
        double clampedAng = MathUtil.clampAngle(moduleAngle);
        double targetRotation = state.angle.getRadians();
        double diff = getClosestAngle(clampedAng, targetRotation);
        double targetAng = moduleAngle + diff;

        double targetSpeed = state.speedMetersPerSecond;
        if (!MathUtil.isWithinTolerance(targetRotation, MathUtil.clampAngle(targetAng), 0.1)) {
            targetSpeed *= -1;
        }

        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAng));
    }
}
