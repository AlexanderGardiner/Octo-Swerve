package frc.robot.Subsystems.light;


public enum CmdIDSequences {
    ConeHigh(new Animations[] {Animations.ID_BLUE, Animations.ID_GREEN}),
    ConeMid(new Animations[] {Animations.ID_BLUE, Animations.ID_PURPLE}),
    CubeHigh(new Animations[] {Animations.ID_BLUE, Animations.ID_YELLOW}),
    CubeMid(new Animations[] {Animations.ID_BLUE, Animations.ID_BLUE}),
    CollectFloor(new Animations[] {Animations.ID_PURPLE, Animations.ID_GREEN});

    public Animations[] animations;
    private CmdIDSequences(Animations[] animations) {
        this.animations = animations;
    }
}
