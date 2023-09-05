
package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorIDs;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;

public class ArmExtension extends SubsystemBase {
    // TODO: OMFG we need to fix the measurements here, it's so arbitrary
    private CANSparkMax motor;
    private static ArmExtension armExtension;
    public SparkMaxPIDController pidController;
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0.25, 0.01, 0);
    private double gearing = 5.0 / 15.0;
    private double gearTicksToMeters = 5;
    private double gearingToMeters = gearing * gearTicksToMeters;
    public double lastpos = 0;
    private SparkMaxConfig extenConfig = new SparkMaxConfig(
            new SparkMaxStatusFrames(
                    500,
                    500,
                    20,
                    500,
                    500,
                    500,
                    500),
            1000,
            false,
            SparkMaxEncoderType.Relative,
            IdleMode.kBrake,
            30,
            30,
            true,
            false,
            42,
            false,
            new PIDConfig(0.3, 0, 0.2, 0));

    public static ArmExtension getInstance() {
        if (armExtension == null) {
            armExtension = new ArmExtension();
        }
        return armExtension;
    }

    public ArmExtension() {
        this.motor = new CANSparkMax(MotorIDs.ARM_EXTENSION, MotorType.kBrushless);
        SparkMaxSetup.setup(motor, extenConfig);
        pidController = motor.getPIDController();
        setOffset();
    }

    public void setOffset() {
        motor.getEncoder().setPosition(0);
        setPosition(0, false);
    }

    public void setPosition(double position, boolean override) {
        if (position > 75 && !override) {
            position = 75;
        }
        if (position < 0 && !override) {
            position = 0;
        }

        lastpos = position;
        motor.getPIDController().setReference(gearing * position, ControlType.kPosition);
    }

    public void setPosition(ArmPositions armPositions, boolean override) {
        setPosition(armPositions.extension, override);
    }

    public double getPosition() {
        return motor.getEncoder().getPosition() / gearing;
    }

    public void zeroArm() {
        motor.setSmartCurrentLimit(5, 5);
        // motor.setInverted(true);
        // motor.setVoltage(-4);
        // setPosition(-3500, true);
    }

    @Override
    public void periodic() {
        setMotorKf();
    }

    public boolean zeroDone() {
        return motor.getOutputCurrent() > 3;
    }

    public void resetCurrent() {
        motor.setSmartCurrentLimit(extenConfig.getStallCurrentLimit(), extenConfig.getFreeCurrentLimit());
    }

    public double getMotorPos() {
        return motor.getEncoder().getPosition();
    }

    public double getMotorVel() {
        return motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getVelocity();
    }

    public void setMotorKf() {
        // pidController.setFF(feedforward.calculate(getMotorPos(), getMotorVel()));
    }

    public double getMotorKf() {
        return pidController.getFF();
    }
}
