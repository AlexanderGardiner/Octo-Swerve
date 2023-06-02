package frc.robot.Libraries.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Libraries.Util.MathUtil;

public class StateOptimizer {
    /**
     * Calculates the shortest distance between 2 angles
     * 
     * @param src    The inital angle
     * @param target The target angle
     * @return The shortest difference between the angles
     */
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

    /**
     * Gets the smallest difference between two angles including equivalent angles
     * 
     * @param clampedAngle The inital angle
     * @param target       The target angle
     * @return The smallest difference between two angles
     */
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

    /**
     * Optimizes the swerve states to take the most efficient route when turning
     * 
     * @param state       The target state of a module
     * @param moduleAngle The current angle of the module
     * @return An optimized state
     */
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
