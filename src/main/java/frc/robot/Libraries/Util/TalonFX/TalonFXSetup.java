package frc.robot.Libraries.Util.TalonFX;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TalonFXSetup {
    public static void setup(WPI_TalonFX talonFX, TalonFXConfig talonFXConfig) {
        if (talonFXConfig.getReset()) {
            talonFX.configFactoryDefault();
        }
        for (int i=0; i<talonFXConfig.getStatusFrames().getFrames().size(); i++) {
            talonFX.setStatusFramePeriod(StatusFrameEnhanced.values()[i], talonFXConfig.getStatusFrames().getFrames().get(i));
        }

        talonFX.configAllowableClosedloopError(0, talonFXConfig.getAllowableClosedLoopError());
        talonFX.configSelectedFeedbackSensor(talonFXConfig.getFeedbackDevice(), 0, 10);

        if (talonFXConfig.getIntegratedSensorInitializationStrategy()!=null) {
            talonFX.configIntegratedSensorInitializationStrategy(talonFXConfig.getIntegratedSensorInitializationStrategy());
        }

        talonFX.setNeutralMode(talonFXConfig.getNeutralMode());

        talonFX.setInverted(talonFXConfig.getInverted());
        talonFX.setSensorPhase(talonFXConfig.getSensorInverted());

        talonFX.configStatorCurrentLimit(talonFXConfig.getStatorCurrentLimit());
        talonFX.configSupplyCurrentLimit(talonFXConfig.getSupplyCurrentLimit());

        talonFX.configMotionCruiseVelocity(talonFXConfig.getMaxVel());
        talonFX.configMotionAcceleration(talonFXConfig.getMaxAccel());
        talonFX.configMotionSCurveStrength(talonFXConfig.getSCurveStrength());

        if (talonFXConfig.getIntegralZone()!=null) {
            talonFX.config_IntegralZone(0, talonFXConfig.getIntegralZone());
        }

        talonFX.config_kP(0, talonFXConfig.getPIDconfig().getP());
        talonFX.config_kI(0, talonFXConfig.getPIDconfig().getI());
        talonFX.config_kD(0, talonFXConfig.getPIDconfig().getD());
        talonFX.config_kF(0, talonFXConfig.getPIDconfig().getF());
        
    }   
}
