package frc.robot.Libraries.Swerve.Util;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;
import frc.robot.Libraries.Util.TalonFX.TalonFXConfig;
import frc.robot.Libraries.Util.TalonFX.TalonFXSetup;
import frc.robot.Libraries.Util.TalonFX.TalonFXStatusFrames;

public class TurnMotor {
    private MotorType motorType;

    private WPI_TalonFX talonFX;
    private CANSparkMax sparkMax;

    private int encoderCountsPerRev;
    
    private boolean simulated;
    private double simulatedEncoderPositionTicks;

    /** Creates a turn motor object
     * @param motorType The type of the motor
     * @param canID The canID of the motor
     * @param PIDconfig The PIDconfig of the motor
     * @param encoderCountsPerRev The ticks per revolution of the encoder
     * @param motorInverted Whether the motor is inverted
     * @param encoderInverted Whether the encoder is is in phase (inverted)
     * @param simulated Whether the motor is simulated
     */
    public TurnMotor(MotorType motorType, int canID, PIDConfig PIDconfig, int encoderCountsPerRev, boolean motorInverted, boolean encoderInverted, boolean simulated) {
        this.motorType = motorType;
        this.encoderCountsPerRev = encoderCountsPerRev;
        this.simulated = simulated;

        if (this.motorType == MotorType.TalonFX) {
            talonFX = new WPI_TalonFX(canID);

            ArrayList<Integer> list = new ArrayList<Integer>();
            list.add(1);
            TalonFXConfig talonFXConfig = new TalonFXConfig(
                new TalonFXStatusFrames(100, 10, 10, 100, 10, 100, 10, 100, 100, 10),
                true,
                0,
                TalonFXFeedbackDevice.IntegratedSensor,
                NeutralMode.Brake,
                new StatorCurrentLimitConfiguration(true, 25, 30, 50),
                new SupplyCurrentLimitConfiguration(true, 30, 30, 50),
                motorInverted,
                encoderInverted,
                encoderCountsPerRev,
                PIDconfig);


            TalonFXSetup.setup(talonFX, talonFXConfig);

        } else if (this.motorType == MotorType.SparkMax) {
            sparkMax = new CANSparkMax(canID, CANSparkMaxLowLevel.MotorType.kBrushless);

            SparkMaxConfig sparkMaxConfig = new SparkMaxConfig(
                new SparkMaxStatusFrames(100, 100, 10, 100, 10, 10, 100),
                1000,
                true,
                SparkMaxEncoderType.Absolute,
                IdleMode.kBrake,
                30,
                30,
                motorInverted,
                encoderInverted,
                encoderCountsPerRev,
                true,
                PIDconfig);


            SparkMaxSetup.setup(sparkMax, sparkMaxConfig);
        }
    }

    /** Gets the encoder position
     * @return The encoder position in ticks
     */
    public double getEncoderPositionTicks() {
        if (this.simulated) {
            return this.simulatedEncoderPositionTicks;
        }

        if (this.motorType == MotorType.TalonFX) {
            return talonFX.getSensorCollection().getIntegratedSensorAbsolutePosition();         
        } else {
            return sparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        }
    }

    /** Sets the target position of the motor
     * @param position The target position in ticks
     */
    public void setTargetPositionTicks(double position) {
        if (this.simulated) {
            this.simulatedEncoderPositionTicks = position;
        } else {
            if (this.motorType == MotorType.TalonFX) {
                talonFX.set(ControlMode.Position, position);
            } else {
                sparkMax.getPIDController().setReference(position, ControlType.kPosition);
            }
        }   
    }
}
