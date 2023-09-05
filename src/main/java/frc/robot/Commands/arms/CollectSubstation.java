package frc.robot.Commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
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

public class CollectSubstation extends CommandBase{
    private ArmPivot armPivot;
    private ArmExtension armExtension;
    private ArmWrist armWrist;
    private ArmRollers armRollers;
    private HippoWrist hippoWrist;
    private HippoRollers hippoRollers;
    private Light light;
    
    public CollectSubstation() {
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
        armRollers.setSpeed(ArmSpeeds.COLLECT);
        armPivot.setAngle(ArmPositions.INTAKE_SUBSTATION);
        armWrist.setAngle(ArmPositions.INTAKE_SUBSTATION);
        armExtension.setPosition(ArmPositions.STOW,false);
        light.command = true;
        light.setAnimation(CmdIDSequences.CollectSubstation);

    }

    @Override
    public void end(boolean interrupted){
        light.command = false;
    }
}
