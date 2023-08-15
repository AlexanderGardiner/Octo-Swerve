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
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class PathPlannerAutos {
    // The starting gyro angle in degrees (CW positive)
    public static double startingGyroAngle;

    /**
     * Creates a map of commands that pathplanner can trigger from within a path
     */
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
            Map.entry("LogTest", new InstantCommand(() -> SmartDashboard.putString("Log", "Test")))));

    /**
     * Creates the pathplanner autobuilder that generates the paths
     */
    public static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            SwerveDrive.getInstance()::getAllianceRelativePose2d,
            SwerveDrive.getInstance()::resetAllianceRelativePose2d,
            new PIDConstants(1, 0, 0),
            new PIDConstants(1.2, 0, 0),
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
    public static CommandBase TestPath() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("TestingPath",
                    new PathConstraints(1, 0.5));

            startingGyroAngle = 0;
            SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());

            SwerveDrive.getInstance().resetGyro();

            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

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
    public static CommandBase TestPath1() {
        return new InstantCommand(() -> {
            List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("Test Path1",
                    new PathConstraints(0.2, 0.1));

            startingGyroAngle = 180;

            SwerveDrive.getInstance().resetGyro();

            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                SwerveDrive.getInstance()
                        .setPoseEstimatorPose2d(new Pose2d(pathgroup.get(0).getInitialHolonomicPose().getX(),
                                pathgroup.get(0).getInitialHolonomicPose().getY(),
                                new Rotation2d(Math.toRadians(MathUtil.flipAngleOverYAxis(startingGyroAngle)))));
                SmartDashboard.putNumber("flipped gyro",
                        MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
                SwerveDrive.getInstance()
                        .setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
            } else {
                SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());
                SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
            }

            CommandScheduler.getInstance().schedule(autoBuilder.fullAuto(pathgroup));
        });
    }
}
