package frc.robot.Subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmZero extends CommandBase{
    
    private ArmExtension armExtension;

    public ArmZero() {
        this.armExtension = ArmExtension.getInstance();
        addRequirements(armExtension);
    }

    @Override
    public void initialize() {
        armExtension.zeroArm();
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
    }
    
}
