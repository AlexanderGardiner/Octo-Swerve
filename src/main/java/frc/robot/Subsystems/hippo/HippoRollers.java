package frc.robot.Subsystems.hippo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class HippoRollers extends SubsystemBase{
    private HippoRollers armRollers;
    public HippoRollers getInstance() {
        if (armRollers == null) {
            armRollers = new HippoRollers();
        }
        return armRollers;
    }

    private CANSparkMax motor;
    private SparkMaxConfig rollerConfig = new SparkMaxConfig(
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
            SparkMaxEncoderType.Relative,
            IdleMode.kBrake,
            20,
            35,
            false,
            false,
            2048,
            false,
            new PIDConfig(0, 0, 0, 0)
    );

    public HippoRollers() {
        motor = new CANSparkMax(MotorIDs.ARM_WRIST_ANGLE, MotorType.kBrushless);
        SparkMaxSetup.setup(motor, rollerConfig);
    }

    public void setSpeed(double speed) {
        motor.setVoltage(speed);
    }
}
