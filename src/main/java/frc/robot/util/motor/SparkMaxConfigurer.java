package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkMaxConfigurer {
    public static CANSparkMax configSpark(int id, SparkMaxConfiguration config) {
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
        config.config(spark);
        Logger.recordOutput("SparkFlashes/" + id, config.getFlashed());
        return spark;
    }
}