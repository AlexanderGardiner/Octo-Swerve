package frc.robot.Commands.Autonomous;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class PathPlannerAutos {
    // The starting gyro angle in degrees (CW positive)
    public static double startingGyroAngle;

    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
            Map.entry("LogTest", new InstantCommand(() -> SmartDashboard.putString("Log", "Test")))));

    public static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            SwerveDrive.getInstance()::getAllianceRelativePose2d,
            SwerveDrive.getInstance()::resetAllianceRelativePose2d,
            new PIDConstants(5, 0, 0),
            new PIDConstants(2, 0, 0),
            SwerveDrive.getInstance()::drive,
            eventMap,
            true,
            SwerveDrive.getInstance());

    public static CommandBase TestPath() {
        startingGyroAngle = 90;

        List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup("Test Path", new PathConstraints(4, 3));
        SwerveDrive.getInstance().setPoseEstimatorPose2d(pathgroup.get(0).getInitialHolonomicPose());
        return autoBuilder.fullAuto(pathgroup);
    }
}
