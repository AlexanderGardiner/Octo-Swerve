package frc.robot.Subsystems.hippo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class HippoWrist extends SubsystemBase{
    
    private HippoWrist hippoWrist;
    public HippoWrist getInstance() {
        if (hippoWrist == null) {
            hippoWrist = new HippoWrist();
        }
        return hippoWrist;
    }

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
            false,
            0,
            false,
            new PIDConfig(2, 0.000, 6, 0.02)
    );

    public HippoWrist() {
        motor = new CANSparkMax(MotorIDs.SPATULA_ANGLE, MotorType.kBrushless);
        SparkMaxSetup.setup(motor, wristConfig);
    }

    public void setSpatulaPos(double angle) {
        if (angle > 0.46) {
            angle = 0.46;
        } else if (angle < 0.12) {
            angle = 0.12;
        }
        motor.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public double getSpatulaPos() {
        return motor.getEncoder().getPosition();
    }
}
