package frc.robot.Libraries.Util.SparkMax;

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Libraries.Util.PIDConfig;
  
public class SparkMaxConfig {
    private SparkMaxStatusFrames statusFrames;
    private int timeoutMs;
    private boolean reset;

    private SparkMaxEncoderType encoderType;
    private IdleMode idleMode;

    private boolean inverted = false;
    private boolean sensorInverted = false;

    private int stallCurrentLimit;
    private int freeCurrentLimit;

    private boolean positionPIDWrappingEnabled = false;

    private double maxVel = 0;
    private double maxAccel = 0;
    private double allowableClosedLoopError = 0;
    private int integralZone = 0;

    private PIDConfig PIDconfig = new PIDConfig(0, 0, 0);

    int encoderCountsPerRev = 42;

    // Default constructor
    public SparkMaxConfig(SparkMaxStatusFrames statusFrames, int timeoutMs, boolean reset,
                          SparkMaxEncoderType encoderType, IdleMode idleMode,
                          int stallCurrentLimit, int freeCurrentLimit,
                          boolean inverted, boolean sensorInverted,
                          int encoderCountsPerRev) {
        this.statusFrames = statusFrames;
        this.timeoutMs = timeoutMs;
        this.reset = reset;

        this.encoderType = encoderType;
        this.idleMode = idleMode;

        this.stallCurrentLimit = stallCurrentLimit;
        this.freeCurrentLimit = freeCurrentLimit;

        this.inverted = inverted;
        this.sensorInverted = sensorInverted;

        this.encoderCountsPerRev = encoderCountsPerRev;
    }

    // Default constructor
    public SparkMaxConfig(SparkMaxStatusFrames statusFrames, int timeoutMs, boolean reset,
                          SparkMaxEncoderType encoderType, IdleMode idleMode,
                          int stallCurrentLimit, int freeCurrentLimit,
                          boolean inverted, boolean sensorInverted,
                          int encoderCountsPerRev,
                          boolean positionPIDWrappingEnabled) {
          this(statusFrames, timeoutMs, reset, 
               encoderType, idleMode,
               stallCurrentLimit, freeCurrentLimit,
               inverted, sensorInverted,
               encoderCountsPerRev);

          this.positionPIDWrappingEnabled = positionPIDWrappingEnabled;
    }

    // PID Control
    public SparkMaxConfig(SparkMaxStatusFrames statusFrames, int timeoutMs, boolean reset,
                          SparkMaxEncoderType encoderType, IdleMode idleMode,
                          int stallCurrentLimit, int freeCurrentLimit,
                          boolean inverted, boolean sensorInverted,
                          int encoderCountsPerRev,
                          boolean positionPIDWrappingEnabled,
                          PIDConfig PIDconfig) {
        this(statusFrames, timeoutMs, reset, 
             encoderType, idleMode,
             stallCurrentLimit, freeCurrentLimit,
             inverted, sensorInverted,
             encoderCountsPerRev,
             positionPIDWrappingEnabled);

        this.PIDconfig = PIDconfig;

    }

    // Constructor for smart motion control
    public SparkMaxConfig(SparkMaxStatusFrames statusFrames, int timeoutMs, boolean reset, 
                          SparkMaxEncoderType encoderType, IdleMode idleMode,
                          int stallCurrentLimit, int freeCurrentLimit,
                          boolean inverted, boolean sensorInverted,
                          int encoderCountsPerRev,
                          boolean positionPIDWrappingEnabled,
                          PIDConfig PIDconfig,
                          Double maxVel, Double maxAccel, double allowableClosedLoopError, Integer integralZone) {
        this(statusFrames, timeoutMs, reset, 
             encoderType, idleMode,
             stallCurrentLimit, freeCurrentLimit,
             inverted, sensorInverted,
             encoderCountsPerRev,
             positionPIDWrappingEnabled,
             PIDconfig);

        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.allowableClosedLoopError = allowableClosedLoopError;
        this.integralZone = integralZone;
    }
    

     public SparkMaxStatusFrames getStatusFrames() {
          return this.statusFrames;
     }

     public void setStatusFrames(SparkMaxStatusFrames statusFrames) {
          this.statusFrames = statusFrames;
     }

     public int getTimeoutMs() {
          return this.timeoutMs;
     }

     public void setTimeoutMs(int timeoutMs) {
          this.timeoutMs = timeoutMs;
     }

     public boolean isReset() {
          return this.reset;
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

     public SparkMaxEncoderType getEncoderType() {
          return this.encoderType;
     }

     public void setEncoderType(SparkMaxEncoderType encoderType) {
          this.encoderType = encoderType;
     }

     public IdleMode getIdleMode() {
          return this.idleMode;
     }

     public void setIdleMode(IdleMode idleMode) {
          this.idleMode = idleMode;
     }

     public boolean isInverted() {
          return this.inverted;
     }

     public boolean getInverted() {
          return this.inverted;
     }

     public void setInverted(boolean inverted) {
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

     public int getStallCurrentLimit() {
          return this.stallCurrentLimit;
     }

     public void setStallCurrentLimit(int stallCurrentLimit) {
          this.stallCurrentLimit = stallCurrentLimit;
     }

     public int getFreeCurrentLimit() {
          return this.freeCurrentLimit;
     }

     public void setFreeCurrentLimit(int freeCurrentLimit) {
          this.freeCurrentLimit = freeCurrentLimit;
     }

     public boolean getPositionPIDWrappingEnabled() {
          return this.positionPIDWrappingEnabled;
     }

     public void setPositionPIDWrappingEnabled(boolean positionPIDWrappingEnabled) {
          this.positionPIDWrappingEnabled = positionPIDWrappingEnabled;
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

     public int getIntegralZone() {
          return this.integralZone;
     }

     public void setIntegralZone(int integralZone) {
          this.integralZone = integralZone;
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
 