package frc.robot.Commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.arm.ArmExtension;
import frc.robot.Subsystems.light.CmdIDSequences;
import frc.robot.Subsystems.light.Light;

public class ArmZero extends CommandBase{
    
    private ArmExtension armExtension;
    private Light light;

    public ArmZero() {
        this.armExtension = ArmExtension.getInstance();
        this.light = Light.getInstance();
        addRequirements(armExtension);
    }

    @Override
    public void initialize() {
        armExtension.zeroArm();
        light.setAnimation(CmdIDSequences.ZeroArm);
    }

    @Override
    public void execute() {
        armExtension.setPosition(-1000, true);
    }

    @Override
    public boolean isFinished() {
        return armExtension.zeroDone();
    }

    @Override
    public void end(boolean interrupted) {
        armExtension.resetCurrent();
        armExtension.setOffset();
    }
    
}
