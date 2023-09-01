package frc.robot.Subsystems.hippo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class HippoWrist extends SubsystemBase{
    
    private static HippoWrist hippoWrist;
    public static HippoWrist getInstance() {
        if (hippoWrist == null) {
            hippoWrist = new HippoWrist();
        }
        return hippoWrist;
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
            true,
            false,
            4096,
            false,
            new PIDConfig(1.1, 0.005, 0, 0)
    );

    public HippoWrist() {
        motor = new CANSparkMax(MotorIDs.SPATULA_ANGLE, MotorType.kBrushless);
        SparkMaxSetup.setup(motor, wristConfig);
    }

    public void setAngle(double angle) {
        MathUtil.clamp(angle, 0.12, 0.46);
        lastpos = angle;
        motor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void setAngle(HippoPositions hippoPositions) {
        setAngle(hippoPositions.angle);
    }

    public double getAngle() {
        return motor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
}
