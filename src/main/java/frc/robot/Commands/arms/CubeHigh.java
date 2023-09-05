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

public class CubeHigh extends CommandBase {

    private ArmPivot armPivot;
    private ArmExtension armExtension;
    private ArmWrist armWrist;
    private ArmRollers armRollers;
    private HippoWrist hippoWrist;
    private HippoRollers hippoRollers;
    private Light light;

    public CubeHigh() {
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
        // not neccessary but it is courtious to our teammates. Hold the cube and begin
        // moving.
        hippoWrist.setAngle(HippoPositions.STOW);
        armRollers.setSpeed(ArmSpeeds.HOLD_CUBE);
        armPivot.setAngle(ArmPositions.CUBE_PLACE_HIGH);
        armExtension.setPosition(ArmPositions.STOW, false);
        start = Timer.getFPGATimestamp();
        light.command = true;
        light.setAnimation(CmdIDSequences.CubeHigh);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("flag", flag);

        switch (flag) {
            case 0:
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armPivot.getAngle(), ArmPositions.CUBE_PLACE_HIGH.armAngle,
                        0.1);

                if (timeout || tolerance) {
                    armExtension.setPosition(ArmPositions.CUBE_PLACE_HIGH, false);
                    armWrist.setAngle(ArmPositions.CUBE_PLACE_HIGH);

                    start = Timer.getFPGATimestamp();
                    flag = 1;

                    if (timeout) {
                        light.setAnimation(Animations.CHECK_FAILED);
                    } else {
                        light.setAnimation(Animations.CHECK_PASSED);
                    }
                }

                break;
            case 1: // Once the extension and wrist and pivot we release the cube
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armWrist.getAngle(), ArmPositions.CUBE_PLACE_MID.wrist, 0.3)
                        && MathUtil.isWithinTolerance(armExtension.getPosition(),
                                ArmPositions.CUBE_PLACE_MID.extension, 5)
                        && MathUtil.isWithinTolerance(armPivot.getAngle(), ArmPositions.CUBE_PLACE_MID.armAngle,
                                0.2);

                if (timeout || tolerance) {
                    armRollers.setSpeed(ArmSpeeds.PLACE_CUBE);
                    armExtension.setPosition(ArmPositions.STOW, false);

                    start = Timer.getFPGATimestamp();
                    flag = 2;

                    if (timeout) {
                        light.setAnimation(Animations.CHECK_FAILED);
                    } else {
                        light.setAnimation(Animations.CHECK_PASSED);
                    }
                }
                break;
            case 2: // The arm clears the obstructions at this point, so we can immediately drop
                    // both the arm and tray table into the stowed and upright position and we're
                    // clear for takeoff.
                timeout = 5 < Timer.getFPGATimestamp() - start;
                tolerance = MathUtil.isWithinTolerance(armExtension.getPosition(),
                        ArmPositions.STOW.extension,
                        0.1);

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
        return flag == 2;
    }

    @Override
    public void end(boolean interrupted) {
        light.command = false;
    }
}
