package frc.robot.Commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.arm.ArmExtension;
import frc.robot.Subsystems.arm.ArmPivot;
import frc.robot.Subsystems.arm.ArmPositions;
import frc.robot.Subsystems.arm.ArmRollers;
import frc.robot.Subsystems.arm.ArmSpeeds;
import frc.robot.Subsystems.arm.ArmWrist;
import frc.robot.Subsystems.hippo.HippoPositions;
import frc.robot.Subsystems.hippo.HippoRollers;
import frc.robot.Subsystems.hippo.HippoWrist;
import frc.robot.Subsystems.light.CmdIDSequences;
import frc.robot.Subsystems.light.Light;

public class PositionAutoAlign extends CommandBase{
    
    private ArmPivot armPivot;
    private ArmExtension armExtension;
    private ArmWrist armWrist;
    private ArmRollers armRollers;
    private HippoWrist hippoWrist;
    private HippoRollers hippoRollers;
    private Light light;

    
    public PositionAutoAlign() {
        //Setup the subsystems. We may want to release the hippo here if a neccessary circumstance can be hypothesized.
        armPivot = ArmPivot.getInstance();
        armExtension = ArmExtension.getInstance();
        armWrist = ArmWrist.getInstance();
        armRollers = ArmRollers.getInstance();
        hippoWrist = HippoWrist.getInstance();
        hippoRollers = HippoRollers.getInstance();
        light = Light.getInstance();
        addRequirements(armPivot, armExtension, armWrist, armRollers, hippoWrist, hippoRollers);
    }

    @Override
    public void initialize() { 
        hippoWrist.setAngle(HippoPositions.STOW);
        hippoRollers.setSpeed(HippoPositions.STOW);
        armRollers.setSpeed(ArmSpeeds.HOLD_STOW_EITHER);
        armPivot.setAngle(ArmPositions.AUTO_ALIGN);
        armExtension.setPosition(ArmPositions.AUTO_ALIGN, false);
        armWrist.setAngle(ArmPositions.AUTO_ALIGN);
        light.command = true;
        light.setAnimation(CmdIDSequences.PositionAutoAlign);

    }

    @Override
    public void end(boolean interrupted){
        light.command = false;
    }
}
