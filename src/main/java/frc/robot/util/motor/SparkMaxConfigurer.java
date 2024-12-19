package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SparkMaxConfigurer {
    public enum EncoderConfigType {
        UVW,
        QUADRATURE
    }

    public static SparkMax configSpark(int id, SparkMaxConfig config) {
        SparkMax spark = new SparkMax(id, MotorType.kBrushless);
        boolean flash = getIfFlash(spark, config);
        spark.configure(
            config, ResetMode.kResetSafeParameters, flash 
                    ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
        Logger.recordOutput("SparkFlashes/" + id, flash);
        return spark;
    }

    /**
     * Return a new, configured {@code SparkMax} object with the given id and parameters.
     * 
     * @param id CAN ID for the spark.
     * @param idleMode idle behavior (brake or coast).
     * @param inverted invert direction?
     * @param smartCurrentLimit current limit in amps.
     * @param encoderMeasurementPeriod time inteval (ms) over which to calculate encoder velocity.
     * @param encoderAverageDepth number of samples taken 
     * @param encoderType
     * @param canTimeout
     * @param statusFrames
     * @return new {@code SparkMax} with the given parameters
     */
    public static SparkMax configSpark(int id, IdleMode idleMode, boolean inverted, int smartCurrentLimit,
            int encoderMeasurementPeriod, int encoderAverageDepth, EncoderConfigType encoderType,
            int canTimeout, StatusFrames statusFrames) {
        SparkMaxConfig config = getConfig(
            idleMode, inverted, smartCurrentLimit, encoderMeasurementPeriod,
            encoderAverageDepth, encoderType, canTimeout, statusFrames);
        return configSpark(id, config);
    }

    public static SparkMaxConfig getConfig(IdleMode idleMode, boolean inverted, int smartCurrentLimit,
            int encoderMeasurementPeriod, int encoderAverageDepth, EncoderConfigType encoderType,
            int canTimeout, StatusFrames statusFrames) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        config.inverted(inverted);
        config.smartCurrentLimit(smartCurrentLimit);
        if (encoderType == EncoderConfigType.UVW) {
            config.encoder.uvwAverageDepth(encoderAverageDepth);
            config.encoder.uvwMeasurementPeriod(encoderMeasurementPeriod);
        } else if (encoderType == EncoderConfigType.QUADRATURE) {
            config.encoder.quadratureAverageDepth(encoderAverageDepth);
            config.encoder.quadratureMeasurementPeriod(encoderMeasurementPeriod);
        }
        // TODO can timeout
        statusFrames.apply(config.signals);
        return config;
    }

    public static boolean getIfFlash(SparkMax spark, SparkMaxConfig config) {
        boolean flash = false;
        return flash;
    }
}