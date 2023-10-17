package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class ArmWrist extends SubsystemBase {

    private static ArmWrist armWrist;

    public static ArmWrist getInstance() {
        if (armWrist == null) {
            armWrist = new ArmWrist();
        }
        return armWrist;
    }

    public double lastpos;
    private CANSparkMax motor;
    private SparkMaxConfig wristConfig = new SparkMaxConfig(
            new SparkMaxStatusFrames(
                    500,
                    20,
                    500,
                    500,
                    500,
                    20,
                    500),
            1000,
            true,
            SparkMaxEncoderType.Absolute,
            IdleMode.kCoast,
            35,
            35,
            false,
            true,
            4096,
            false,
            new PIDConfig(2, 0.000, 6, 0.02));

    public ArmWrist() {
        motor = new CANSparkMax(MotorIDs.ARM_WRIST_ANGLE, MotorType.kBrushless);
        SparkMaxSetup.setup(motor, wristConfig);
    }

    public void setAngle(double angle) {
        lastpos = angle;
        // SmartDashboard.putNumber("angle", angle);
        motor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void setAngle(ArmPositions armPositions) {
        setAngle(armPositions.wrist);
    }

    public double getAngle() {
        return motor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
}
