package frc.robot;

import java.lang.management.ManagementFactory;

import com.pathplanner.lib.server.PathPlannerServer;
import com.sun.management.OperatingSystemMXBean;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Autonomous.PathPlannerAutos;
import frc.robot.Commands.Swerve.SwerveControl;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d field = new Field2d();
  private double teleopTimer;
  private SendableChooser<Command> autoChooser;

  private OperatingSystemMXBean osBean = ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);

  /**
   * Runs on robot startup
   * <ul>
   * <li>Initalizes Autonomous Selector and field2d</li>
   * <li>Starts pathplanner server</li>
   * </ul>
   */
  @Override
  public void robotInit() {
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Test Path", PathPlannerAutos.TestPath());
    autoChooser.addOption("Test Path1", PathPlannerAutos.TestPath1());

    SmartDashboard.putData("autonomous", autoChooser);

    SmartDashboard.putData("Field", field);
    PathPlannerServer.startServer(5811);

  }

  /**
   * Runs every robot loop
   * <ul>
   * <li>Updates logging</li>
   * </ul>
   */
  @Override
  public void robotPeriodic() {
    CANStatus canBus = new CANStatus();
    SmartDashboard.putNumber("CAN-USAGE", canBus.percentBusUtilization);
    SmartDashboard.putNumber("RIO-CPU", osBean.getCpuLoad());
    SmartDashboard.putNumber("RIO-RAM",
        (((double) osBean.getTotalMemorySize() - (double) osBean.getFreeMemorySize())
            / (double) osBean.getTotalMemorySize()));
    double startTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      SmartDashboard.putBoolean("IsRed", true);
    } else {
      SmartDashboard.putBoolean("IsRed", false);
    }

    SmartDashboard.putNumber("Time for loop (ms)", (Timer.getFPGATimestamp() - startTime) * 1000);

    field.setRobotPose(SwerveDrive.getInstance().getPose2d());
  }

  /**
   * Runs when the robot is disabled
   * <ul>
   * <li>Logs that the robot is disabled
   * </ul>
   */
  @Override
  public void disabledInit() {
    SmartDashboard.putString("Time-Left", "Robot Disabled");
  }

  /**
   * Runs every robot loop when the robot is disabled
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * Runs when the robot exits disabled mode
   */
  @Override
  public void disabledExit() {
  }

  /**
   * Runs when the robot enters autonomous mode
   * <ul>
   * <li>Cancels current commands</li>
   * <li>Removes default commands</li>
   * <li>Runs selected auto command</li>
   */
  @Override
  public void autonomousInit() {
    SmartDashboard.putString("Time-Left", "Robot In Auto");
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().removeDefaultCommand(SwerveDrive.getInstance());

    m_autonomousCommand = autoChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Runs every robot loop when the robot is in autonomous
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * Runs when the robot exists autonomous smode
   */
  @Override
  public void autonomousExit() {
  }

  /**
   * Runs when the robot the robot enters teleop
   * <ul>
   * <li>Resets gyro if not coming from autonomous</li>
   * <li>Sets the target pose2d for the drive pid loop</li>
   * <li>Starts default commands</li>
   * </ul>
   */
  @Override
  public void teleopInit() {
    teleopTimer = Timer.getFPGATimestamp();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      SwerveDrive.getInstance().setTargetPose2d(SwerveDrive.getInstance().getPose2d());
    } else {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        SwerveDrive.getInstance().resetGyro();
        SwerveDrive.getInstance().setGyroAngleAdjustment(180);
        SwerveDrive.getInstance().setPoseEstimatorGyroOffset(new Rotation2d());
        SwerveDrive.getInstance().setPoseEstimatorPose2d(new Pose2d(0,0,new Rotation2d(0)));
        SwerveDrive.getInstance().setTargetPose2d(new Pose2d(0,0,new Rotation2d(Math.PI)));

      } else {
        SwerveDrive.getInstance().resetGyro();
        SwerveDrive.getInstance().setGyroAngleAdjustment(0);
        SwerveDrive.getInstance().setPoseEstimatorGyroOffset(new Rotation2d());
        SwerveDrive.getInstance().setPoseEstimatorPose2d(new Pose2d(0,0,new Rotation2d(0)));
        SwerveDrive.getInstance().setTargetPose2d(new Pose2d(0,0,new Rotation2d(0)));

      }
    }

    SwerveDrive.getInstance().updatePose();

    

    CommandScheduler.getInstance().setDefaultCommand(SwerveDrive.getInstance(), new SwerveControl());
  }

  /**
   * Runs every robot loop when the robot is in teleop
   * <ul>
   * <li>Logs the time left in teleop</li>
   * </l>
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Time-Left", Double.valueOf(135 - (Timer.getFPGATimestamp() - teleopTimer)).toString());
  }

  /**
   * Runs when the robot exits teleop
   */
  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().removeDefaultCommand(SwerveDrive.getInstance());
  }
}
