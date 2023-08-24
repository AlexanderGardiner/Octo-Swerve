package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.arms.CollectFloor;
import frc.robot.Commands.arms.ConeHigh;
import frc.robot.Commands.arms.ConeMid;
import frc.robot.Commands.arms.CubeHigh;
import frc.robot.Commands.arms.CubeMid;

public class ButtonConfig {
    public static void initTeleop() {
        //DRIVER 6 is gyro reset
        //DRIVER 13 is driver assist, should be removed

        //DRIVER_RIGHT 1 is auto align

        //DRIVER_LEFT 1 is 0 degrees
        //DRIVER_LEFT 2 is 180 degrees

        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).onTrue(new CollectFloor());
        //CODRIVER 2 is substation collect
        //CODRIVER 3 is drive/stow both
        //CODRIVER 4 is hippo intake
        //CODRIVER 5 is hippo place
        //CODRIVER 6 is arm zero
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7).onTrue(new CubeHigh());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8).onTrue(new ConeHigh());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9).onTrue(new CubeMid());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10).onTrue(new ConeMid());
        //CODRIVER 11 is arm auto align position
        //CODRIVER 12 is stow both
    }
}
