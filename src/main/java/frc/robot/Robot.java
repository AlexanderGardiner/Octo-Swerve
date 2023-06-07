package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Autonomous.PathPlannerAutos;
import frc.robot.Commands.Swerve.SwerveControl;
import frc.robot.Libraries.Util.MathUtil;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d field = new Field2d();

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", field);
    PathPlannerServer.startServer(5811);
  }

  @Override
  public void robotPeriodic() {

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
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().removeDefaultCommand(SwerveDrive.getInstance());

    m_autonomousCommand = PathPlannerAutos.TestPath();
    SwerveDrive.getInstance().resetGyro();

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      SwerveDrive.getInstance().setGyroAngleAdjustment(MathUtil.flipAngleOverYAxis(PathPlannerAutos.startingGyroAngle));
    } else {
      SwerveDrive.getInstance().setGyroAngleAdjustment(PathPlannerAutos.startingGyroAngle);
    }

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
