package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.arms.CollectFloor;
import frc.robot.Commands.arms.ConeHigh;
import frc.robot.Commands.arms.ConeMid;
import frc.robot.Commands.arms.CubeHigh;
import frc.robot.Commands.arms.CubeMid;

public class ButtonConfig {
    public static void initTeleop() { //these are wrong, please assign them properly. TODO: This will be redone later.
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).onTrue(new ConeHigh());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2).onTrue(new ConeMid());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3).onTrue(new CubeHigh());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4).onTrue(new CubeMid());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5).onTrue(new CollectFloor());


    }
}
