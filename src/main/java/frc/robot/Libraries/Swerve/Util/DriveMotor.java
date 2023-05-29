package frc.robot.Libraries.Swerve.Util;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Libraries.Util.PIDConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxConfig;
import frc.robot.Libraries.Util.SparkMax.SparkMaxEncoderType;
import frc.robot.Libraries.Util.SparkMax.SparkMaxSetup;
import frc.robot.Libraries.Util.SparkMax.SparkMaxStatusFrames;
import frc.robot.Libraries.Util.TalonFX.TalonFXConfig;
import frc.robot.Libraries.Util.TalonFX.TalonFXSetup;
import frc.robot.Libraries.Util.TalonFX.TalonFXStatusFrames;

public class DriveMotor {
    private MotorType motorType;

    private WPI_TalonFX talonFX;
    private CANSparkMax sparkMax;

    private int encoderCountsPerRev;

    private boolean simulated;
    private double simulatedEncoderPositionTicks = 0;
    private double simulatedEncoderVelocityTicksPer100ms;
    private double lastTimeSimulatedEncoderPositionUpdated = 0;

    /** Creates a drive motor object
     * @param motorType The type of the motor
     * @param canID The canID of the motor
     * @param PIDconfig The PIDconfig of the motor
     * @param encoderCountsPerRev The ticks per revolution of the encoder
     * @param motorInverted Whether the motor is inverted
     * @param encoderInverted Whether the encoder is is in phase (inverted)
     */
    public DriveMotor(MotorType motorType, int canID, PIDConfig PIDconfig, int encoderCountsPerRev, boolean motorInverted, boolean encoderInverted, boolean simulated) {
        this.motorType = motorType;
        this.encoderCountsPerRev = encoderCountsPerRev;
        this.simulated = simulated;

        if (simulated) {
            lastTimeSimulatedEncoderPositionUpdated = Timer.getFPGATimestamp();
        }

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
                new StatorCurrentLimitConfiguration(true, 55, 30, 50),
                new SupplyCurrentLimitConfiguration(true, 60, 30, 50),
                motorInverted,
                encoderInverted,
                encoderCountsPerRev,
                PIDconfig);


            TalonFXSetup.setup(talonFX, talonFXConfig);

        } else if (this.motorType == MotorType.SparkMax) {
            sparkMax = new CANSparkMax(canID, null);

            SparkMaxConfig sparkMaxConfig = new SparkMaxConfig(
                new SparkMaxStatusFrames(100, 100, 10, 100, 10, 10, 100),
                1000,
                true,
                SparkMaxEncoderType.Alternate,
                IdleMode.kBrake,
                30,
                60,
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
            this.simulatedEncoderPositionTicks += simulatedEncoderVelocityTicksPer100ms * (Timer.getFPGATimestamp() - lastTimeSimulatedEncoderPositionUpdated)*10.0;
            lastTimeSimulatedEncoderPositionUpdated = Timer.getFPGATimestamp();
            
            return this.simulatedEncoderPositionTicks;
        }

        if (this.motorType == MotorType.TalonFX) {
            return talonFX.getSensorCollection().getIntegratedSensorPosition();
        } else {
            return sparkMax.getAlternateEncoder(encoderCountsPerRev).getPosition();
        }
    }

    /** Gets the encoder velocity
     * @return The encoder velocity in ticks per 100ms
     */
    public double getEncoderVelocityTicks() {
        if (this.simulated) {
            return this.simulatedEncoderVelocityTicksPer100ms;
        }

        if (this.motorType == MotorType.TalonFX) {
            return talonFX.getSensorCollection().getIntegratedSensorVelocity();
        } else {
            return (sparkMax.getAlternateEncoder(encoderCountsPerRev).getVelocity())/600.0;
        }
    }

    /** Sets the target velocity of the motor
     * @param velocity The target velocity in ticks per 100ms
     */
    public void setTargetVelocityTicks(double velocity) {
        if (this.simulated) {
            this.simulatedEncoderPositionTicks += simulatedEncoderVelocityTicksPer100ms * (Timer.getFPGATimestamp() - lastTimeSimulatedEncoderPositionUpdated)*10.0;
            this.simulatedEncoderVelocityTicksPer100ms = velocity;    

            lastTimeSimulatedEncoderPositionUpdated = Timer.getFPGATimestamp();
        } else {
            if (this.motorType == MotorType.TalonFX) {
                talonFX.set(ControlMode.Velocity, velocity);
            } else {
                sparkMax.getPIDController().setReference((velocity*600), ControlType.kVelocity);
            }
        } 
    }
}
