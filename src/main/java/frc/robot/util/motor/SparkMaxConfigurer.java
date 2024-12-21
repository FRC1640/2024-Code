package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SparkMaxConfigurer {

    public static SparkMax configSpark(int id, IdleMode idleMode, boolean inverted, int smartCurrentLimit,
            int encoderMeasurementPeriod, int encoderAverageDepth, int canTimeout, StatusFrames statusFrames) {
        SparkMax spark = new SparkMax(id, MotorType.kBrushless);
        boolean flash =
            (inverted != spark.getInverted()) ||
            (idleMode != spark.configAccessor.getIdleMode()) ||
            (smartCurrentLimit != spark.configAccessor.getSmartCurrentLimit()) ||
            (encoderMeasurementPeriod != spark.configAccessor.encoder.getQuadratureMeasurementPeriod()) ||
            (encoderAverageDepth != spark.configAccessor.encoder.getQuadratureAverageDepth()) ||
            (statusFrames.getFlashNecessary(spark));
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        config.inverted(inverted);
        config.smartCurrentLimit(smartCurrentLimit);
        config.encoder.quadratureAverageDepth(encoderAverageDepth).quadratureMeasurementPeriod(encoderMeasurementPeriod);
        statusFrames.apply(config.signals);
        spark.configure(
            config, ResetMode.kResetSafeParameters, flash
                    ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
        Logger.recordOutput("SparkFlashes/" + id, flash);
        return spark;
    }
}