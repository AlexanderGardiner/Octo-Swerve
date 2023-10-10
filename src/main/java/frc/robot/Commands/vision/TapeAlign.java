package frc.robot.Commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    // private Light light;

    public TapeAlign() {
        // Initialization
        this.swerveDrive = SwerveDrive.getInstance();
        this.vision = Vision.getInstance();
        // this.caliGirls = CaliGirls.getInstance();
        this.cameraToTarget = null;
        // this.light = Light.getInstance();
    }

    @Override
    public void initialize() {
        // try {
        if (vision.getTapeCamHasTarget()) {
            if (vision.getBestTarget() != null) {
                cameraToTarget = vision.getBestTarget();
                // caliGirls.setBottomPos(ArmPositions.AUTO_ALIGN.armAngle);
                //caliGirls.setBottomKf();
            } else {
                // end(true);
            }
        }
        // } catch (Exception e) {
        //     // TO DO: handle exception
        // }
    }

    public int detectioncycles = 0;

    @Override
    public void execute() {
        try {
            if (vision.getTapeCamHasTarget()) {
                if (vision.getBestTarget() != null) {
                    cameraToTarget = vision.getBestTarget();
                    //     // SmartDashboard.putNumber("Degrees", cameraToTarget.getYaw()-3);
                    //     //light.setDegrees(cameraToTarget.getYaw()-3);
                    //     //light.setHasTarget(true);
                    //     //light.lightUpdateControl(-1);
                    ySpeed = (cameraToTarget.getYaw() - 1.5) * 0.1;
                    //     // SmartDashboard.putNumber("Degrees to target", cameraToTarget.getYaw()-3);

                        if (MathUtil.isWithinTolerance(cameraToTarget.getYaw() - 1.5, 0, 2)) {
                            Light.getInstance().setAnimation(Animations.ALIGNED);
                        } else {
                            Light.getInstance().setAnimation(Animations.ALIGNMENT);
                        }

                } else {
                    ySpeed = 0;
                    //     //light.setHasTarget(false);
                    //     //light.lightUpdateControl(-1);
                    //     light.AdrUpdateStrobe(255, 0, 0, 1);
                    Light.getInstance().setAnimation(Animations.ALIGNMENT);
                    // end(true);
                    //     // driveTrain.drive(0, 0, 0, true);
                }
                swerveDrive.drive(new ChassisSpeeds(0, ySpeed, 0), true);
                // //SmartDashboard.putNumber("Y_SPED", ySpeed);
                // CommandScheduler.getInstance().schedule(new SetDriveAngle(180));
                
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
