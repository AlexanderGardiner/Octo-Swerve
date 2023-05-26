package frc.robot.Libraries.Util;

public class MathUtil {
    /**
     * <p>Wraps a circular angle value to within one circle.</p>
     *
     * <p>Customizable number of angle units per circle. Angle is
     * equivalent and wrapped to the positive [0, fullCircle] range.</p>
     *
     * @param angle the raw angle units to wrap.
     * @param fullCircle the number of angle units to be one circle.
     * @return the wrapped angle in corresponding angle units.
     */
    public static double wrapToCircle(double angle, double fullCircle) {
        angle %= fullCircle;
        return angle < 0 ? fullCircle + angle : angle;
    }

    /**
     * Checks if a value is within a certain tolerance of a target. Directions irrelevant.
     *
     * @param value the current value for which to check.
     * @param target the target to check the value against.
     * @param tolerance the tolerance (positive and negative directions)
     *                  around the target that is acceptable error
     *                  for the value to be "within tolerance".
     * @return if the value is within tolerance of the target.
     */
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    public static double clampAngle(double angle) {
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}
