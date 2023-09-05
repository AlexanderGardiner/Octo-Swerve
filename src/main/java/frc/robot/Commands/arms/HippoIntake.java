package frc.robot.Commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.hippo.HippoPositions;
import frc.robot.Subsystems.hippo.HippoRollers;
import frc.robot.Subsystems.hippo.HippoWrist;
import frc.robot.Subsystems.light.CmdIDSequences;
import frc.robot.Subsystems.light.Light;

public class HippoIntake extends CommandBase{
    
    private HippoWrist hippoWrist;
    private HippoRollers hippoRollers;
    private Light light;

    public HippoIntake() {
        hippoWrist = HippoWrist.getInstance();
        hippoRollers = HippoRollers.getInstance();
        light = Light.getInstance();
        addRequirements(hippoWrist, hippoRollers);
    }

    @Override
    public void initialize() { 
        hippoWrist.setAngle(HippoPositions.INTAKE);
        hippoRollers.setSpeed(HippoPositions.INTAKE);
        light.command = true;
        light.setAnimation(CmdIDSequences.HippoIntake);
    }

    @Override
    public void end(boolean interrupted){
        light.command = false;
    }
}
