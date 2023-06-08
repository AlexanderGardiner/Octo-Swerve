package frc.robot;

import java.lang.management.ManagementFactory;

import com.pathplanner.lib.server.PathPlannerServer;
import com.sun.management.OperatingSystemMXBean;

import edu.wpi.first.hal.can.CANStatus;
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

  @Override
  public void robotInit() {
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Test Path", PathPlannerAutos.TestPath());
    autoChooser.addOption("Test Path1", PathPlannerAutos.TestPath1());

    SmartDashboard.putData("autonomous", autoChooser);

    SmartDashboard.putData("Field", field);
    PathPlannerServer.startServer(5811);

  }

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
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Time-Left", "Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

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

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    teleopTimer = Timer.getFPGATimestamp();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        SwerveDrive.getInstance().setGyroAngleAdjustment(180);
      }
      SwerveDrive.getInstance().resetGyro();
    }

    SwerveDrive.getInstance().setTargetPose2d(SwerveDrive.getInstance().getPose2d());

    CommandScheduler.getInstance().setDefaultCommand(SwerveDrive.getInstance(), new SwerveControl());
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Time-Left", Double.valueOf(135 - (Timer.getFPGATimestamp() - teleopTimer)).toString());
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
    field.setRobotPose(SwerveDrive.getInstance().getPose2d());
  }
}
