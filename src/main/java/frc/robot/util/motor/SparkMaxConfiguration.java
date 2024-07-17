package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.SparkMaxDefaults;

public class SparkMaxConfiguration {

    private IdleMode idleMode;
    private boolean inverted;
    private int smartCurrentLimit;
    private int encoderMeasurementPeriod;
    private int encoderAverageDepth;
    private int canTimeout;
    private List<LimitSwitchConfiguration> limitSwitches;
    private StatusFrames statusFrames;
    private boolean burn = false;

    public SparkMaxConfiguration(IdleMode idleMode, boolean inverted, int smartCurrentLimit,
            int encoderMeasurementPeriod, int encoderAverageDepth, int canTimeout, StatusFrames statusFrames) {
        this.idleMode = idleMode;
        this.inverted = inverted;
        this.smartCurrentLimit = smartCurrentLimit;
        this.encoderMeasurementPeriod = encoderMeasurementPeriod;
        this.encoderAverageDepth = encoderAverageDepth;
        this.canTimeout = canTimeout;
        this.limitSwitches = new ArrayList<>();
        this.limitSwitches.add(SparkMaxDefaults.limitSwitch);
        this.statusFrames = statusFrames;
    }

    public SparkMaxConfiguration(IdleMode idleMode, boolean inverted, int smartCurrentLimit, int encoderMeasurementPeriod,
            int encoderAverageDepth, int canTimeout, StatusFrames statusFrames, LimitSwitchConfiguration ... limitSwitches) {
        this.idleMode = idleMode;
        this.inverted = inverted;
        this.smartCurrentLimit = smartCurrentLimit;
        this.encoderMeasurementPeriod = encoderMeasurementPeriod;
        this.encoderAverageDepth = encoderAverageDepth;
        this.canTimeout = canTimeout;
        this.limitSwitches = new ArrayList<>();
        for (LimitSwitchConfiguration switchConfig : limitSwitches) {
            this.limitSwitches.add(switchConfig);
        }
        this.statusFrames = statusFrames;
    }

    private void configIdleMode(CANSparkMax spark) {
        if (spark.getIdleMode() != idleMode) {
            spark.setIdleMode(idleMode);
            burn = true;
        }
    }

    private void configInverted(CANSparkMax spark) {
        if (spark.getInverted() != inverted) {
            spark.setInverted(inverted);
            burn = true;
        }
    }

    private void configSmartCurrentLimit(CANSparkMax spark) {
        spark.setSmartCurrentLimit(smartCurrentLimit);
    }

    private void configEncoderMeasurementPeriod(CANSparkMax spark) {
        if (spark.getEncoder().getMeasurementPeriod() != encoderMeasurementPeriod) {
            spark.getEncoder().setMeasurementPeriod(encoderMeasurementPeriod);
            burn = true;
        }
    }

    private void configEncoderAverageDepth(CANSparkMax spark) {
        if (spark.getEncoder().getAverageDepth() != encoderAverageDepth) {
            spark.getEncoder().setAverageDepth(encoderAverageDepth);
            burn = true;
        }
    }

    private void configCANTimeout(CANSparkMax spark) {
        spark.setCANTimeout(canTimeout);
    }

    private void configLimitSwitches(CANSparkMax spark) {
        for (LimitSwitchConfiguration switchConfig : limitSwitches) {
            if (switchConfig.differentFrom(spark)) {
                switchConfig.apply(spark);
                burn = true;
            }
        }
    }

    private void updateStatusFrames(CANSparkMax spark) {
        statusFrames.updateStatusFrames(spark);
    }

    private void burnFlash(CANSparkMax spark) {
        spark.burnFlash();
    }

    public boolean getFlashed() {
        return burn;
    }

    public void config(CANSparkMax spark) {
        configIdleMode(spark);
        configInverted(spark);
        configSmartCurrentLimit(spark);
        configEncoderMeasurementPeriod(spark);
        configEncoderAverageDepth(spark);
        configCANTimeout(spark);
        configLimitSwitches(spark);
        updateStatusFrames(spark);
        if (burn) { burnFlash(spark); }
    }
}