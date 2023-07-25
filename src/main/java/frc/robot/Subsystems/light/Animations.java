package frc.robot.Subsystems.light;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public enum Animations {
    
    DISABLE(new StrobeAnimation(255, 255, 255, 0, 1, 60), 0),
    ENABLE(new RainbowAnimation(), 0),
    COMMAND_ACTIVE(new StrobeAnimation(255, 0, 255, 0, 0, 60), 0),

    CHECK_FAILED(new StrobeAnimation(255, 0, 0, 0, 0.5, 60), 0.1),
    CHECK_PASSED(new StrobeAnimation(0, 255, 0, 0, 0.5, 60), 0.1),
    BOOT_COMPLETE(new RainbowAnimation(), 0.5),
    
    ALIGNMENT(new StrobeAnimation(255, 0, 0, 0, 0.5, 60), 0),
    ALIGNED(new StrobeAnimation(0, 100, 255, 0, 0, 60), 0),
    BALANCING(new StrobeAnimation(255, 0, 0, 0, 1, 60), 0),
    BALANCED(new StrobeAnimation(0, 255, 0, 0, 0, 60), 0),

    ID_YELLOW(new StrobeAnimation(255, 255, 0, 0, 1, 60), 0.08),
    ID_GREEN(new StrobeAnimation(0, 255, 0, 0, 1, 60), 0.08),
    ID_BLUE(new StrobeAnimation(0, 0, 255, 0, 1, 60), 0.08),
    ID_PURPLE(new StrobeAnimation(255, 0, 255, 0, 1, 60), 0.08);


    public Animation animation;
    public double time;
    private Animations(Animation animation, double time) {
        this.animation = animation;
        this.time = time;
    }

    
}
