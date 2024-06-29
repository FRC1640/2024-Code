package frc.robot.util.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.CAN;

public class SparkMaxConfiguration {

    private IdleMode idleMode;
    private boolean inverted;
    private boolean limitSwitch;
    private Type limSwitchType;
    private int smartCurrentLimit;
    private int encoderMeasurementPeriod;
    private int encoderAverageDepth;
    private int canTimeout;

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
        spark.setIdleMode(idleMode);
    }

    public void configInverted(CANSparkMax spark) {
        spark.setInverted(inverted);
    }

    public void configLimitSwitch(CANSparkMax spark) {
        spark.getReverseLimitSwitch(limSwitchType).enableLimitSwitch(limitSwitch);
    }

    public void configSmartCurrentLimit(CANSparkMax spark) {
        spark.setSmartCurrentLimit(smartCurrentLimit);
    }

    public void configEncoderMeasurementPeriod(CANSparkMax spark) {
        spark.getEncoder().setMeasurementPeriod(encoderMeasurementPeriod);
    }

    public void configEncoderAverageDepth(CANSparkMax spark) {
        spark.getEncoder().setAverageDepth(encoderAverageDepth);
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
        burnFlash(spark);
    }
}