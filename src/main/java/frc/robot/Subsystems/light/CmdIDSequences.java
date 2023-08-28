package frc.robot.Subsystems.light;

import frc.robot.Commands.arms.HippoIntake;

public enum CmdIDSequences {
    ConeHigh(new Animations[] {Animations.ID_BLUE, Animations.ID_GREEN}),
    ConeMid(new Animations[] {Animations.ID_BLUE, Animations.ID_PURPLE}),
    CubeHigh(new Animations[] {Animations.ID_BLUE, Animations.ID_YELLOW}),
    CubeMid(new Animations[] {Animations.ID_BLUE, Animations.ID_BLUE}),
    CollectFloor(new Animations[] {Animations.ID_PURPLE, Animations.ID_GREEN}),
    CollectSubstation(new Animations[] {Animations.ID_PURPLE, Animations.ID_YELLOW}),
    HippoIntake(new Animations[] {Animations.ID_YELLOW, Animations.ID_GREEN}),
    HippoPlace(new Animations[] {Animations.ID_YELLOW, Animations.ID_PURPLE}),
    PositionDrive(new Animations[] {Animations.ID_GREEN, Animations.ID_PURPLE}),
    PositionStow(new Animations[] {Animations.ID_GREEN, Animations.ID_YELLOW}),
    PositionAutoAlign(new Animations[] {Animations.ID_GREEN, Animations.ID_BLUE});

    public Animations[] animations;
    private CmdIDSequences(Animations[] animations) {
        this.animations = animations;
    }
}
