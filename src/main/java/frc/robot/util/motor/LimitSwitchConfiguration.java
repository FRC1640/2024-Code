package frc.robot.util.motor;

import java.util.function.BiFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants.SparkMaxDefaults;

public class LimitSwitchConfiguration {

    private BiFunction<CANSparkMax, Type, SparkLimitSwitch> getSwitch;
    private Type switchType;
    private boolean enable;

    public LimitSwitchConfiguration(LimitSwitchReverse direction, Type switchType, boolean enable) {
        getSwitch = SparkMaxDefaults.getLimitSwitch.get(direction);
        this.switchType = switchType;
        this.enable = enable;
    }

    public void apply(CANSparkMax spark) {
        getSwitch.apply(spark, switchType).enableLimitSwitch(enable);
    }

    public boolean differentFrom(CANSparkMax spark) {
        return getSwitch.apply(spark, switchType).isLimitSwitchEnabled() != enable;
    }

    public enum LimitSwitchReverse {
        kForward,
        kReverse
    }
}