package frc.robot.util.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;

public class SparkMaxConfiguration {

    private IdleMode idleMode;
    private boolean inverted;
    private boolean limitSwitch;
    private Type limSwitchType;
    private int smartCurrentLimit;
    private int encoderMeasurementPeriod;
    private int encoderAverageDepth;
    private int canTimeout;
    private boolean burn = false;

    public SparkMaxConfiguration(IdleMode idleMode, boolean inverted,
            boolean limitSwitch, Type limSwitchType, int smartCurrentLimit, int encoderMeasurementPeriod,
            int encoderAverageDepth, int canTimeout) {
        this.idleMode = idleMode;
        this.inverted = inverted;
        this.limitSwitch = limitSwitch;
        this.limSwitchType = limSwitchType;
        this.smartCurrentLimit = smartCurrentLimit;
        this.encoderMeasurementPeriod = encoderMeasurementPeriod;
        this.encoderAverageDepth = encoderAverageDepth;
        this.canTimeout = canTimeout;
    }

    public void configIdleMode(CANSparkMax spark) {
        if (spark.getIdleMode() != idleMode) {
            spark.setIdleMode(idleMode);
            burn = true;
        }
    }

    public void configInverted(CANSparkMax spark) {
        if (spark.getInverted() != inverted) {
            spark.setInverted(inverted);
            burn = true;
        }
    }

    public void configLimitSwitch(CANSparkMax spark) {
        if (spark.getReverseLimitSwitch(limSwitchType).isLimitSwitchEnabled() != limitSwitch) {
            spark.getReverseLimitSwitch(limSwitchType).enableLimitSwitch(limitSwitch);
            burn = true;
        }
    }

    public void configSmartCurrentLimit(CANSparkMax spark) {
        spark.setSmartCurrentLimit(smartCurrentLimit);
    }

    public void configEncoderMeasurementPeriod(CANSparkMax spark) {
        if (spark.getEncoder().getMeasurementPeriod() != encoderMeasurementPeriod) {
            spark.getEncoder().setMeasurementPeriod(encoderMeasurementPeriod);
            burn = true;
        }
    }

    public void configEncoderAverageDepth(CANSparkMax spark) {
        if (spark.getEncoder().getAverageDepth() != encoderAverageDepth) {
            spark.getEncoder().setAverageDepth(encoderAverageDepth);
            burn = true;
        }
    }

    public void configCANTimeout(CANSparkMax spark) {
        spark.setCANTimeout(canTimeout);
    }

    public void burnFlash(CANSparkMax spark) {
        spark.burnFlash();
    }

    public void config(CANSparkMax spark, Type limSwitchType) {
        configIdleMode(spark);
        configInverted(spark);
        configLimitSwitch(spark);
        configSmartCurrentLimit(spark);
        configEncoderMeasurementPeriod(spark);
        configEncoderAverageDepth(spark);
        configCANTimeout(spark);
        if (burn) { burnFlash(spark); }
    }
}