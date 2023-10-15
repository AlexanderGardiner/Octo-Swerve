package frc.robot.Subsystems.arm;

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

public class ArmRollers extends SubsystemBase {
    private static ArmRollers armRollers;

    public static ArmRollers getInstance() {
        if (armRollers == null) {
            armRollers = new ArmRollers();
        }
        return armRollers;
    }

    private CANSparkMax motor;
    private SparkMaxConfig rollerConfig = new SparkMaxConfig(
            new SparkMaxStatusFrames(
                    500,
                    100,
                    500,
                    500,
                    500,
                    500,
                    500),
            1000,
            true,
            SparkMaxEncoderType.Relative,
            IdleMode.kBrake,
            10,
            30,
            false,
            false,
            2048,
            false,
            new PIDConfig(0, 0, 0, 0));

    public ArmRollers() {
        motor = new CANSparkMax(MotorIDs.ARM_ROLLER, MotorType.kBrushless);
        SparkMaxSetup.setup(motor, rollerConfig);
    }

    public void setSpeed(double speed) {
        motor.setVoltage(speed);
    }

    public void setSpeed(ArmSpeeds armSpeeds) {
        setSpeed(armSpeeds.roller);
    }
}
