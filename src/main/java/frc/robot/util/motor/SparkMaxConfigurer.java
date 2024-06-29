package frc.robot.util.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

public class SparkMaxConfigurer {
    public static CANSparkMax configSpark(int id, SparkMaxConfiguration config, Type limSwitchType) {
        CANSparkMax spark = new CANSparkMax(id, MotorType.kBrushless);
        config.config(spark, limSwitchType);
        return spark;
    }
}