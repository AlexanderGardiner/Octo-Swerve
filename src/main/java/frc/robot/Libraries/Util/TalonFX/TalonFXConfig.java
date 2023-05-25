package frc.robot.Libraries.Util.TalonFX;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Libraries.Util.PIDConfig;
  
public class TalonFXConfig {
    private TalonFXStatusFrames statusFrames;
    private boolean reset;
    private double allowableClosedLoopError;

    private TalonFXFeedbackDevice feedbackDevice;
    private SensorInitializationStrategy integratedSensorInitializationStrategy = null;
    private NeutralMode neutralMode;

    private TalonFXInvertType inverted = TalonFXInvertType.Clockwise;
    private boolean sensorInverted = false;

    private StatorCurrentLimitConfiguration statorCurrentLimit;
    private SupplyCurrentLimitConfiguration supplyCurrentLimit;

    private double maxVel = 0;
    private double maxAccel = 0;
    private Integer integralZone = null;
    private int sCurveStrength = 0;

    private PIDConfig PIDconfig = new PIDConfig(0, 0, 0);

    int encoderCountsPerRev = 4096;

    // Default constructor
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError, 
                         TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
                         StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
                         TalonFXInvertType inverted, boolean sensorInverted,
                         int encoderCountsPerRev) {
        this.statusFrames = statusFrames;
        this.reset = reset;
        this.allowableClosedLoopError = allowableClosedLoopError;

        this.feedbackDevice = feedbackDevice;

        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;

        this.inverted = inverted;
        this.sensorInverted = sensorInverted;

        this.encoderCountsPerRev = encoderCountsPerRev;
    }

    // PID Control
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError, 
                         TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
                         StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
                         TalonFXInvertType inverted, boolean sensorInverted,
                         int encoderCountsPerRev,
                         PIDConfig PIDconfig) {
        this(statusFrames, reset, allowableClosedLoopError, 
             feedbackDevice, neutralMode,
             statorCurrentLimit, supplyCurrentLimit,
             inverted, sensorInverted,
             encoderCountsPerRev);

        this.PIDconfig = PIDconfig;

    }

    // Constructor for motion magic control
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError, 
                         TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
                         StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
                         TalonFXInvertType inverted, boolean sensorInverted,
                         int encoderCountsPerRev,
                         PIDConfig PIDconfig,
                         Double maxVel, Double maxAccel, Integer integralZone, Integer sCurveStrength) {
        this(statusFrames, reset, allowableClosedLoopError, 
             feedbackDevice, neutralMode,
             statorCurrentLimit, supplyCurrentLimit,
             inverted, sensorInverted,
             encoderCountsPerRev,
             PIDconfig);

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.integralZone = integralZone;
        this.sCurveStrength = sCurveStrength;
    }

    // PID Control (Integrated Sensor)
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError, 
                         TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
                         StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
                         TalonFXInvertType inverted, boolean sensorInverted,
                         int encoderCountsPerRev,
                         PIDConfig PIDconfig,
                         SensorInitializationStrategy integratedSensorInitializationStrategy) {
        this(statusFrames, reset, allowableClosedLoopError, 
             feedbackDevice, neutralMode,
             statorCurrentLimit, supplyCurrentLimit,
             inverted, sensorInverted,
             encoderCountsPerRev,
             PIDconfig);

        this.integratedSensorInitializationStrategy = integratedSensorInitializationStrategy;

    }

    // Motion magic control (Integrated Sensor)
    public TalonFXConfig(TalonFXStatusFrames statusFrames, boolean reset, double allowableClosedLoopError, 
                         TalonFXFeedbackDevice feedbackDevice, NeutralMode neutralMode,
                         StatorCurrentLimitConfiguration statorCurrentLimit, SupplyCurrentLimitConfiguration supplyCurrentLimit,
                         TalonFXInvertType inverted, boolean sensorInverted,
                         int encoderCountsPerRev,
                         PIDConfig PIDconfig,
                         SensorInitializationStrategy integratedSensorInitializationStrategy,
                         Double maxVel, Double maxAccel, Integer integralZone, Integer sCurveStrength) {
        this(statusFrames, reset, allowableClosedLoopError, 
             feedbackDevice, neutralMode,
             statorCurrentLimit, supplyCurrentLimit,
             inverted, sensorInverted,
             encoderCountsPerRev,
             PIDconfig,
             integratedSensorInitializationStrategy);

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.integralZone = integralZone;
        this.sCurveStrength = sCurveStrength;
    }


    public TalonFXStatusFrames getStatusFrames() {
        return this.statusFrames;
    }

    public void setStatusFrames(TalonFXStatusFrames statusFrames) {
        this.statusFrames = statusFrames;
    }

    public boolean getReset() {
        return this.reset;
    }

    public void setReset(boolean reset) {
        this.reset = reset;
    }

    public double getAllowableClosedLoopError() {
        return this.allowableClosedLoopError;
    }

    public void setAllowableClosedLoopError(double allowableClosedLoopError) {
        this.allowableClosedLoopError = allowableClosedLoopError;
    }

    public TalonFXFeedbackDevice getFeedbackDevice() {
        return this.feedbackDevice;
    }

    public void setFeedbackDevice(TalonFXFeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
    }

    public SensorInitializationStrategy getIntegratedSensorInitializationStrategy() {
        return this.integratedSensorInitializationStrategy;
    }

    public void setIntegratedSensorInitializationStrategy(SensorInitializationStrategy integratedSensorInitializationStrategy) {
        this.integratedSensorInitializationStrategy = integratedSensorInitializationStrategy;
    }

    public NeutralMode getNeutralMode() {
        return this.neutralMode;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    public TalonFXInvertType getInverted() {
        return this.inverted;
    }

    public void setInverted(TalonFXInvertType inverted) {
        this.inverted = inverted;
    }

    public boolean isSensorInverted() {
        return this.sensorInverted;
    }

    public boolean getSensorInverted() {
        return this.sensorInverted;
    }

    public void setSensorInverted(boolean sensorInverted) {
        this.sensorInverted = sensorInverted;
    }

    public StatorCurrentLimitConfiguration getStatorCurrentLimit() {
        return this.statorCurrentLimit;
    }

    public void setStatorCurrentLimit(StatorCurrentLimitConfiguration statorCurrentLimit) {
        this.statorCurrentLimit = statorCurrentLimit;
    }

    public SupplyCurrentLimitConfiguration getSupplyCurrentLimit() {
        return this.supplyCurrentLimit;
    }

    public void setSupplyCurrentLimit(SupplyCurrentLimitConfiguration supplyCurrentLimit) {
        this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getMaxVel() {
        return this.maxVel;
    }

    public void setMaxVel(double maxVel) {
        this.maxVel = maxVel;
    }

    public double getMaxAccel() {
        return this.maxAccel;
    }

    public void setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
    }

    public Integer getIntegralZone() {
        return this.integralZone;
    }

    public void setIntegralZone(int integralZone) {
        this.integralZone = integralZone;
    }

    public int getSCurveStrength() {
        return this.sCurveStrength;
    }

    public void setSCurveStrength(int sCurveStrength) {
        this.sCurveStrength = sCurveStrength;
    }

    public PIDConfig getPIDconfig() {
        return this.PIDconfig;
    }

    public void setPIDconfig(PIDConfig PIDconfig) {
        this.PIDconfig = PIDconfig;
    }
    
    public int getEncoderCountsPerRev() {
        return this.encoderCountsPerRev;
    }

    public void setEncoderCountsPerRev(int encoderCountsPerRev) {
        this.encoderCountsPerRev = encoderCountsPerRev;
    }


}
 