package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants.SparkMaxDefaults;

public class SparkMaxConfiguration {

    private IdleMode idleMode;
    private boolean inverted;
    private int smartCurrentLimit;
    private int encoderMeasurementPeriod;
    private int encoderAverageDepth;
    private int canTimeout;
    private StatusFrames statusFrames;
    private boolean burn = false;
    private Map<Double, Type> limitSwitches;
    private List<BiFunction<CANSparkMax, Type, SparkLimitSwitch>> getLimitSwitch = new ArrayList<>();


    public SparkMaxConfiguration(IdleMode idleMode, boolean inverted, int smartCurrentLimit,
            int encoderMeasurementPeriod, int encoderAverageDepth, int canTimeout, StatusFrames statusFrames) {
        this.idleMode = idleMode;
        this.inverted = inverted;
        this.smartCurrentLimit = smartCurrentLimit;
        this.encoderMeasurementPeriod = encoderMeasurementPeriod;
        this.encoderAverageDepth = encoderAverageDepth;
        this.canTimeout = canTimeout;
        this.statusFrames = statusFrames;
        this.limitSwitches = Map.of(1.0, SparkLimitSwitch.Type.kNormallyOpen);
        getLimitSwitch.add((CANSparkMax a, Type b) -> a.getForwardLimitSwitch(b));
        getLimitSwitch.add((CANSparkMax c, Type d) -> c.getReverseLimitSwitch(d));
        
    }

    public SparkMaxConfiguration(IdleMode idleMode, boolean inverted, int smartCurrentLimit, int encoderMeasurementPeriod,
            int encoderAverageDepth, int canTimeout, StatusFrames statusFrames, Map<Double, Type> limitSwitches) {
        this.idleMode = idleMode;
        this.inverted = inverted;
        this.smartCurrentLimit = smartCurrentLimit;
        this.encoderMeasurementPeriod = encoderMeasurementPeriod;
        this.encoderAverageDepth = encoderAverageDepth;
        this.canTimeout = canTimeout;
        this.statusFrames = statusFrames;
        this.limitSwitches = limitSwitches;
        getLimitSwitch.add((CANSparkMax a, Type b) -> a.getForwardLimitSwitch(b));
        getLimitSwitch.add((CANSparkMax c, Type d) -> c.getReverseLimitSwitch(d));
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

    public void configLimitSwitches(CANSparkMax spark) {
        for (int i = 0; i < limitSwitches.size(); i++) {
            
        }


        if (spark.getReverseLimitSwitch(limitSwitchType).isLimitSwitchEnabled() != limitSwitch) {
            spark.getReverseLimitSwitch(limitSwitchType).enableLimitSwitch(limitSwitch);
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

    public void updateStatusFrames(CANSparkMax spark) {
        statusFrames.updateStatusFrames(spark);
    }

    public void burnFlash(CANSparkMax spark) {
        spark.burnFlash();
    }

    public boolean getFlashed() {
        return burn;
    }

    public void config(CANSparkMax spark) {
        configIdleMode(spark);
        configInverted(spark);
        configLimitSwitches(spark);
        configSmartCurrentLimit(spark);
        configEncoderMeasurementPeriod(spark);
        configEncoderAverageDepth(spark);
        configCANTimeout(spark);
        updateStatusFrames(spark);
        if (burn) { burnFlash(spark); }
    }
}