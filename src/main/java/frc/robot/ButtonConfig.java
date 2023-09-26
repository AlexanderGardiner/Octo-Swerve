package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Swerve.GyroZero;
import frc.robot.Commands.arms.ArmZero;
import frc.robot.Commands.arms.CollectFloor;
import frc.robot.Commands.arms.CollectSubstation;
import frc.robot.Commands.arms.ConeHigh;
import frc.robot.Commands.arms.ConeMid;
import frc.robot.Commands.arms.CubeHigh;
import frc.robot.Commands.arms.CubeMid;
import frc.robot.Commands.arms.HippoIntake;
import frc.robot.Commands.arms.HippoPlace;
import frc.robot.Commands.arms.PositionAutoAlign;
import frc.robot.Commands.arms.PositionDrive;
import frc.robot.Commands.arms.PositionStow;
import frc.robot.Commands.vision.TapeAlign;

public class ButtonConfig {
    public static void initTeleop() {
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 6).onTrue(new GyroZero());
        //DRIVER 13 is driver assist, should be removed

        //TODO:Write these with the new library
        //DRIVER_RIGHT 1 is auto align
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1).onTrue(new TapeAlign());

        //DRIVER_LEFT 1 is 0 degrees
        //DRIVER_LEFT 2 is 180 degrees

        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).onTrue(new CollectFloor());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2).onTrue(new CollectSubstation());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3).onTrue(new PositionDrive());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4).onTrue(new HippoIntake());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5).onTrue(new HippoPlace());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6).onTrue(new ArmZero());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7).onTrue(new CubeHigh());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8).onTrue(new ConeHigh());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9).onTrue(new CubeMid());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10).onTrue(new ConeMid());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 11).onTrue(new PositionAutoAlign());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 12).onTrue(new PositionStow());
    }
}
