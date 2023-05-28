package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ControlMap {
    public static final Joystick DRIVER_LEFT = new Joystick(0);
    public static final Joystick DRIVER_RIGHT = new Joystick(1);
    public static final Joystick DRIVER_BUTTONS = new Joystick(2);
    public static final Joystick CO_DRIVER_LEFT = new Joystick(3);
    public static final Joystick CO_DRIVER_RIGHT = new Joystick(4);
    public static final Joystick CO_DRIVER_BUTTONS = new Joystick(5);

    private ControlMap() {
    }
}
