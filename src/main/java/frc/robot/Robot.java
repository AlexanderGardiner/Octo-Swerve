package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Autonomous.PathPlannerAutos;
import frc.robot.Commands.Swerve.SwerveControl;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d field = new Field2d();

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    SwerveDrive.getInstance().setPoseEstimatorGyroOffset(new Rotation2d());
    SwerveDrive.getInstance().setGyroAngleAdjustment(0);
    SwerveDrive.getInstance().resetGyro();

    m_autonomousCommand = PathPlannerAutos.TestPath();

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
    SwerveDrive.getInstance().setPoseEstimatorGyroOffset(new Rotation2d());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      SwerveDrive.getInstance().setGyroAngleAdjustment(-PathPlannerAutos.startingGyroAngle);

    } else {
      SwerveDrive.getInstance().resetGyro();
    }

    SwerveDrive.getInstance().setTargetPose2d(SwerveDrive.getInstance().getPose2d());

    // SwerveDrive.getInstance().resetPose2d(new
    // Pose2d(SwerveDrive.getInstance().getPose2d().getX(),
    // SwerveDrive.getInstance().getPose2d().getY(),
    // SwerveDrive.getInstance().getPose2d().getRotation().plus(new
    // Rotation2d(Math.toRadians(PathPlannerAutos.robotInitalRotation)))));;
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
