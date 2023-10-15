package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class ArmPivot extends SubsystemBase {
    private static ArmPivot armPivot;

    public static ArmPivot getInstance() {
        if (armPivot == null) {
            armPivot = new ArmPivot();
        }
        return armPivot;
    }

    private CANSparkMax motor1;
    private CANSparkMax motor2;
    public double lastpos;

    public ArmPivot() {
        SparkMaxConfig pivotConfig1 = new SparkMaxConfig(
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
                5,
                15,
                false,
                true,
                4096,
                false,
                new PIDConfig(6, 0.0, 0, 0.06));
        motor1 = new CANSparkMax(MotorIDs.ARM_PIVOT_ANGLE, MotorType.kBrushless);
        SparkMaxSetup.setup(motor1, pivotConfig1);

        SparkMaxConfig pivotConfig2 = new SparkMaxConfig(
                new SparkMaxStatusFrames(
                        100,
                        100,
                        500,
                        500,
                        500,
                        500,
                        500),
                1000,
                true,
                IdleMode.kCoast,
                5,
                15,
                true,
                motor1);
        motor2 = new CANSparkMax(MotorIDs.ARM_PIVOT_ANGLE_FOLLOWER, MotorType.kBrushless);
        SparkMaxSetup.setup(motor2, pivotConfig2);
        motor1.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(0.72 + (-.258) - (1 / 6) + 0.07);
    }

    public void setAngle(double angle) {
        lastpos = angle;
        motor1.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void setAngle(ArmPositions armPositions) {
        setAngle(armPositions.armAngle);
    }

    public double getAngle() {
        return motor1.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

}
