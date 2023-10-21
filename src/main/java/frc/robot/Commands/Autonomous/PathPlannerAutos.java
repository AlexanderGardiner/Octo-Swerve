package frc.robot.Commands.Autonomous;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.Swerve.BalanceChargeStationAuto;
import frc.robot.Commands.arms.ConeHigh;
import frc.robot.Commands.arms.HippoIntake;
import frc.robot.Commands.arms.HippoPlace;
import frc.robot.Commands.arms.PositionAutoAlign;
import frc.robot.Commands.arms.PositionStow;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.drivetrain.SwerveDrive;

public class PathPlannerAutos {
    // The starting gyro angle in degrees (CW positive)
    public static double startingGyroAngle;

    /**
     * Creates a map of commands that pathplanner can trigger from within a path
     */
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
            Map.entry("LogTest", new InstantCommand(() -> SmartDashboard.putString("Log", "Test"))),
            Map.entry("HippoIntake", new HippoIntake()),
            Map.entry("Stow", new PositionStow()),
            Map.entry("HippoPlace", new HippoPlace()),
            Map.entry("ConeHigh", new ConeHigh()),
            Map.entry("AlignPosition", new PositionAutoAlign()),
            Map.entry("Balance", new BalanceChargeStationAuto())));

    /**
     * Creates the pathplanner autobuilder that generates the paths
     */
    public static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            SwerveDrive.getInstance()::getAllianceRelativePose2d,
            SwerveDrive.getInstance()::resetAllianceRelativePose2d,
            new PIDConstants(0.7, 0, 0),
            new PIDConstants(3.5, 0, 0),
            SwerveDrive.getInstance()::drive,
            eventMap,
            true,
            SwerveDrive.getInstance());

    /**
     * An example path
     * <ul>
     * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
     * make sure 0 is towards the red alliance (cw positive)</li>
     * <li>Sets the inital pose</li>
     * <li>Resets the gyro and offsets it so 0 is facing towards the red
     * alliance</li>
     * <li>Schedules the path</li>
     * </ul>
     * 
     * @return The path as a command
     */
    public static CommandBase BLUECenter2Piece() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("BLUECenter2Piece",
                    new PathConstraints(2, 1));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (false) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

    /**
    * An example path
    * <ul>
    * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
    * make sure 0 is towards the red alliance (cw positive)</li>
    * <li>Sets the inital pose</li>
    * <li>Resets the gyro and offsets it so 0 is facing towards the red
    * alliance</li>
    * <li>Schedules the path</li>
    * </ul>
    * 
    * @return The path as a command
    */
    public static CommandBase REDCenter2Piece() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("REDCenter2Piece",
                    new PathConstraints(2, 1));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (true) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

    /**
     * An example path
     * <ul>
     * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
     * make sure 0 is towards the red alliance (cw positive)</li>
     * <li>Sets the inital pose</li>
     * <li>Resets the gyro and offsets it so 0 is facing towards the red
     * alliance</li>
     * <li>Schedules the path</li>
     * </ul>
     * 
     * @return The path as a command
     */
    public static CommandBase PlaceHighBalance() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("PlaceHighBalance",
                    new PathConstraints(4, 3));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

    /**
     * An example path
     * <ul>
     * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
     * make sure 0 is towards the red alliance (cw positive)</li>
     * <li>Sets the inital pose</li>
     * <li>Resets the gyro and offsets it so 0 is facing towards the red
     * alliance</li>
     * <li>Schedules the path</li>
     * </ul>
     * 
     * @return The path as a command
     */
    public static CommandBase BLUESubstation2Piece() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("BLUESubstation2Piece",
                    new PathConstraints(2, 1));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (false) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

    /**
    * An example path
    * <ul>
    * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
    * make sure 0 is towards the red alliance (cw positive)</li>
    * <li>Sets the inital pose</li>
    * <li>Resets the gyro and offsets it so 0 is facing towards the red
    * alliance</li>
    * <li>Schedules the path</li>
    * </ul>
    * 
    * @return The path as a command
    */
    public static CommandBase REDSubstation2Piece() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("REDSubstation2Piece",
                    new PathConstraints(2, 1));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (true) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

    /**
     * An example path
     * <ul>
     * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
     * make sure 0 is towards the red alliance (cw positive)</li>
     * <li>Sets the inital pose</li>
     * <li>Resets the gyro and offsets it so 0 is facing towards the red
     * alliance</li>
     * <li>Schedules the path</li>
     * </ul>
     * 
     * @return The path as a command
     */
    public static CommandBase BLUECable2Piece() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("BLUECable2Piece",
                    new PathConstraints(2, 1));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (false) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

    /**
     * An example path
     * <ul>
     * <li>Sets the starting gyro angle to the holonmic angle of the inital pose,
     * make sure 0 is towards the red alliance (cw positive)</li>
     * <li>Sets the inital pose</li>
     * <li>Resets the gyro and offsets it so 0 is facing towards the red
     * alliance</li>
     * <li>Schedules the path</li>
     * </ul>
     * 
     * @return The path as a command
     */
    public static CommandBase REDCable2Piece() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("REDCable2Piece",
                    new PathConstraints(2, 1));

            startingGyroAngle = 180;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            if (true) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            SwerveDrive.getInstance().updatePose();

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }

}
