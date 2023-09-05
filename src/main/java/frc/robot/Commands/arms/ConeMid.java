package frc.robot.Commands.arms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.arm.ArmExtension;
import frc.robot.Subsystems.arm.ArmPivot;
import frc.robot.Subsystems.arm.ArmPositions;
import frc.robot.Subsystems.arm.ArmRollers;
import frc.robot.Subsystems.arm.ArmSpeeds;
import frc.robot.Subsystems.arm.ArmWrist;
import frc.robot.Subsystems.hippo.HippoPositions;
import frc.robot.Subsystems.hippo.HippoRollers;
import frc.robot.Subsystems.hippo.HippoWrist;
import frc.robot.Subsystems.light.Animations;
import frc.robot.Subsystems.light.CmdIDSequences;
import frc.robot.Subsystems.light.Light;

public class ConeMid extends CommandBase {

    private ArmPivot armPivot;
    private ArmExtension armExtension;
    private ArmWrist armWrist;
    private ArmRollers armRollers;
    private HippoWrist hippoWrist;
    private HippoRollers hippoRollers;
    private Light light;

    public ConeMid() {
        // Setup the subsystems. We may want to release the hippo here if a neccessary
        // circumstance can be hypothesized.
        armPivot = ArmPivot.getInstance();
        armExtension = ArmExtension.getInstance();
        armWrist = ArmWrist.getInstance();
        armRollers = ArmRollers.getInstance();
        hippoWrist = HippoWrist.getInstance();
        hippoRollers = HippoRollers.getInstance();
        light = Light.getInstance();
        addRequirements(armPivot, armExtension, armWrist, armRollers, hippoWrist, hippoRollers);
        // Initialize flag to zero. This will increment our sequence tracking in the
        // switch case as the movement progresses.
        flag = 0;
    }

    private int flag;
    private double start;

    private boolean timeout;
    private boolean tolerance;

    @Override
    public void initialize() {
        // Start by stowing the hippo, and beginning to raise the arm. RAW, the hippo is
        // not neccessary but it is courtious to our teammates. Hold the cone and begin
        // moving.
        hippoWrist.setAngle(HippoPositions.STOW);
        armRollers.setSpeed(ArmSpeeds.HOLD_CONE);
        armPivot.setAngle(ArmPositions.PRE_CONE_PLACE_MID);
        armExtension.setPosition(ArmPositions.STOW, false);
        start = Timer.getFPGATimestamp();
        light.command = true;
        light.setAnimation(CmdIDSequences.ConeHigh);
        flag = 0;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("flag", flag);

        switch (flag) {
            case 0: // At a certain point of acceptable height, we allow the extension and wrist to
                    // begin moving even before the pivot is done.
                    // This should help speed up the placement, but will need to be tuned carefully.
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armPivot.getAngle(), ArmPositions.HALF_CONE_PLACE_MID.armAngle,
                        0.1);

                if (timeout || tolerance) {
                    armExtension.setPosition(ArmPositions.PRE_CONE_PLACE_MID, false);
                    armWrist.setAngle(ArmPositions.PRE_CONE_PLACE_MID);

                    start = Timer.getFPGATimestamp();
                    flag = 1;

                    if (timeout) {
                        light.setAnimation(Animations.CHECK_FAILED);
                    } else {
                        light.setAnimation(Animations.CHECK_PASSED);
                    }
                }

                break;
            case 1: // Once the extension and wrist are in position, we are able to both lower the
                    // arm into place, and release the cone.
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armWrist.getAngle(), ArmPositions.PRE_CONE_PLACE_MID.wrist, 0.3)
                        && MathUtil.isWithinTolerance(armExtension.getPosition(),
                                ArmPositions.PRE_CONE_PLACE_MID.extension, 5)
                        && MathUtil.isWithinTolerance(armPivot.getAngle(), ArmPositions.PRE_CONE_PLACE_MID.armAngle,
                                0.2);

                if (timeout || tolerance) {
                    armPivot.setAngle(ArmPositions.CONE_PLACE_MID);
                    armRollers.setSpeed(ArmSpeeds.PLACE_CONE);

                    start = Timer.getFPGATimestamp();
                    flag = 2;

                    if (timeout) {
                        light.setAnimation(Animations.CHECK_FAILED);
                    } else {
                        light.setAnimation(Animations.CHECK_PASSED);
                    }
                }
                break;
            case 2: // After the pivot is in place or at least past that point, we can immediately
                    // bring the arm back and spit the cone out more aggressively.
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armPivot.getAngle(), ArmPositions.CONE_PLACE_MID.armAngle, 0.1);

                if (timeout || tolerance) {
                    armExtension.setPosition(ArmPositions.STOW, false);
                    armRollers.setSpeed(ArmSpeeds.EJECT_CONE);

                    start = Timer.getFPGATimestamp();
                    flag = 3;

                    if (timeout) {
                        light.setAnimation(Animations.CHECK_FAILED);
                    } else {
                        light.setAnimation(Animations.CHECK_PASSED);
                    }
                }
                break;
            case 3: // The arm clears the obstructions at this point, so we can immediately drop
                    // both the arm and tray table into the stowed and upright position and we're
                    // clear for takeoff.
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armExtension.getMotorPos(),
                        ArmPositions.HALF_CONE_PLACE_MID.extension, 5);

                if (timeout || tolerance) {
                    armWrist.setAngle(ArmPositions.STOW);
                    armPivot.setAngle(ArmPositions.STOW);

                    light.setAnimation(Animations.CHECK_PASSED);
                    flag = 4;

                    if (timeout) {
                        light.setAnimation(Animations.CHECK_FAILED);
                    } else {
                        light.setAnimation(Animations.CHECK_PASSED);
                    }
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // After everything is stowed we're done here.
        return flag == 4;
    }

    @Override
    public void end(boolean interrupted) {
        light.command = false;
    }
}
