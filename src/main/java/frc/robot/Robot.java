package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
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
import frc.robot.Subsystems.arm.ArmExtension;
import frc.robot.Subsystems.arm.ArmPivot;
import frc.robot.Subsystems.arm.ArmRollers;
import frc.robot.Subsystems.arm.ArmWrist;
import frc.robot.Subsystems.drivetrain.SwerveDrive;
import frc.robot.Subsystems.hippo.HippoRollers;
import frc.robot.Subsystems.hippo.HippoWrist;
import frc.robot.Subsystems.light.Animations;
import frc.robot.Subsystems.light.Light;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d field = new Field2d();
  private SendableChooser<Command> autoChooser;

  // private OperatingSystemMXBean osBean =
  // ManagementFactory.getPlatformMXBean(OperatingSystemMXBean.class);

  /**
   * Runs on robot startup
   * <ul>
   * <li>Initalizes Autonomous Selector and field2d</li>
   * <li>Starts pathplanner server</li>
   * </ul>
   */
  @Override
  public void robotInit() {
    initAllSubsystems();
    Light.getInstance().setAnimation(Animations.BOOT_COMPLETE);
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Cable Side 2 Piece BLUE", PathPlannerAutos.ConeHighCubeLowCableSide());
    autoChooser.addOption("Substation Side 2 Piece BLUE", PathPlannerAutos.ConeHighCubeLow());
    autoChooser.setDefaultOption("Cable Side 2 Piece RED", PathPlannerAutos.ConeHighCubeLowCableSideRed());
    autoChooser.addOption("Substation Side 2 Piece RED", PathPlannerAutos.ConeHighCubeLowRed());

    SmartDashboard.putData("autonomous", autoChooser);

    SmartDashboard.putData("Field", field);
    PathPlannerServer.startServer(5811);
    ArmExtension.getInstance().setOffset();

  }

  /**
   * Runs every robot loop
   * <ul>
   * <li>Updates logging</li>
   * </ul>
   */
  @Override
  public void robotPeriodic() {
    // CANStatus canBus = new CANStatus();
    // SmartDashboard.putNumber("CAN-USAGE", canBus.percentBusUtilization);
    // SmartDashboard.putNumber("RIO-CPU", osBean.getCpuLoad());
    // SmartDashboard.putNumber("RIO-RAM",
    // (((double) osBean.getTotalMemorySize() - (double) osBean.getFreeMemorySize())
    // / (double) osBean.getTotalMemorySize()));
    double startTime = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();

    // if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    // SmartDashboard.putBoolean("IsRed", true);
    // } else {
    // SmartDashboard.putBoolean("IsRed", false);
    // }

    SmartDashboard.putNumber("Extension position", ArmExtension.getInstance().getPosition());
    SmartDashboard.putNumber("Extension target", ArmExtension.getInstance().lastpos);
    SmartDashboard.putNumber("Pivot position", ArmPivot.getInstance().getAngle());
    SmartDashboard.putNumber("Pivot target", ArmPivot.getInstance().lastpos);
    SmartDashboard.putNumber("Wrist position", ArmWrist.getInstance().getAngle());
    SmartDashboard.putNumber("Wrist target", ArmWrist.getInstance().lastpos);
    SmartDashboard.putNumber("Hippo position", HippoWrist.getInstance().getAngle());
    SmartDashboard.putNumber("Hippo target", HippoWrist.getInstance().lastpos);

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
    Light.getInstance().setAnimation(Animations.DISABLE);

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
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().removeDefaultCommand(SwerveDrive.getInstance());

    SwerveDrive.getInstance().resetGyro();
    Light.getInstance().setAnimation(Animations.ENABLE);

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
    Light.getInstance().setAnimation(Animations.ENABLE);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      SwerveDrive.getInstance().setTargetPose2d(SwerveDrive.getInstance().getPose2d());
    } else {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        SwerveDrive.getInstance().resetGyro();
        SwerveDrive.getInstance().setGyroAngleAdjustment(180);
        SwerveDrive.getInstance().setPoseEstimatorGyroOffset(new Rotation2d());
        SwerveDrive.getInstance().setPoseEstimatorPose2d(new Pose2d(0, 0, new Rotation2d(0)));
        SwerveDrive.getInstance().setTargetPose2d(new Pose2d(0, 0, new Rotation2d(Math.PI)));

      } else {
        SwerveDrive.getInstance().resetGyro();
        SwerveDrive.getInstance().setGyroAngleAdjustment(0);
        SwerveDrive.getInstance().setPoseEstimatorGyroOffset(new Rotation2d());
        SwerveDrive.getInstance().setPoseEstimatorPose2d(new Pose2d(0, 0, new Rotation2d(0)));
        SwerveDrive.getInstance().setTargetPose2d(new Pose2d(0, 0, new Rotation2d(0)));

      }
    }

    SwerveDrive.getInstance().updatePose();

    ButtonConfig.initTeleop();

    CommandScheduler.getInstance().setDefaultCommand(SwerveDrive.getInstance(),
        new SwerveControl());
  }

  /**
   * Runs every robot loop when the robot is in teleop
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * Runs when the robot exits teleop
   */
  @Override
  public void teleopExit() {
    CommandScheduler.getInstance().removeDefaultCommand(SwerveDrive.getInstance());
  }

  private void initAllSubsystems() {
    Light.getInstance();
    ArmExtension.getInstance();
    ArmPivot.getInstance();
    ArmRollers.getInstance();
    ArmWrist.getInstance();
    HippoWrist.getInstance();
    HippoRollers.getInstance();
  }
}
