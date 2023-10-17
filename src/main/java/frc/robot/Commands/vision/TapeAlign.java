package frc.robot.Commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.drivetrain.SwerveDrive;
import frc.robot.Subsystems.light.Animations;
import frc.robot.Subsystems.light.Light;
// import frc.robot.commands.swerve.SetDriveAngle;
// import frc.robot.subsystems.Light;
// import frc.robot.subsystems.arm.ArmPositions;
// import frc.robot.subsystems.arm.CaliGirls;
// import frc.robot.subsystems.swerve.DriveTrain;
// import frc.robot.subsystems.vision.Vision;
// import frc.robot.util.MathUtil;
import frc.robot.Subsystems.vision.Vision;

public class TapeAlign extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Vision vision;
    // private final CaliGirls caliGirls;
    private PhotonTrackedTarget cameraToTarget;
    private double ySpeed = 0;
    private Light light;

    public TapeAlign() {
        // Initialization
        this.swerveDrive = SwerveDrive.getInstance();
        this.vision = Vision.getInstance();
        this.cameraToTarget = null;
        this.light = Light.getInstance();
    }

    @Override
    public void initialize() {
        // try {
        if (vision.getTapeCamHasTarget()) {
            if (vision.getBestTarget() != null) {
                cameraToTarget = vision.getBestTarget();
            }
        }
    }

    @Override
    public void execute() {
        try {
            if (vision.getTapeCamHasTarget()) {
                if (vision.getBestTarget() != null) {
                    cameraToTarget = vision.getBestTarget();
                    if (DriverStation.getAlliance()==DriverStation.Alliance.Blue) {
                        ySpeed = (cameraToTarget.getYaw() - 4) * 0.07;
                    } else {
                        ySpeed = -(cameraToTarget.getYaw() - 4) * 0.07;
                    }
                    
                    if (MathUtil.isWithinTolerance(cameraToTarget.getYaw() - 4, 0, 2)) {
                        light.setAnimation(Animations.ALIGNED);
                    } else {
                        light.setAnimation(Animations.ALIGNMENT);
                    }

                } else {
                    ySpeed = 0;
                    light.setAnimation(Animations.ALIGNMENT);
                }
                swerveDrive.drive(new ChassisSpeeds(swerveDrive.previousXSpeed, ySpeed, 0), true);
            }
        } catch (Exception e) {
            // error
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0), true);
    }

}
