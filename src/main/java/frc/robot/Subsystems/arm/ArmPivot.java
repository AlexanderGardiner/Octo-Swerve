package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class ArmPivot extends SubsystemBase {
    private ArmPivot armPivot;
    public ArmPivot getInstance() {
        if (armPivot == null) {
            armPivot = new ArmPivot();
        }
        return armPivot;
    }

    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private SparkMaxConfig pivotConfig1 = new SparkMaxConfig(
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
            30,
            30,
            false,
            true,
            0,
            false,
            null,
            null,
            null,
            0
        );


}
