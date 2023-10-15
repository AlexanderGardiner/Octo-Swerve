package frc.robot.Commands.arms;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.arm.ArmRollers;
import frc.robot.Subsystems.arm.ArmSpeeds;

public class ClawNone extends InstantCommand {

    ArmRollers rollers;

    public ClawNone() {
        this.rollers = ArmRollers.getInstance();
    }

    @Override
    public void initialize() {
        this.rollers.setSpeed(ArmSpeeds.NOTHING);
    }

}
