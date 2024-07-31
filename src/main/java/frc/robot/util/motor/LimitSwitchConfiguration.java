package frc.robot.util.motor;

import java.util.function.BiFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

public class LimitSwitchConfiguration {

    public enum LimitSwitchDirection {
        kForward,
        kReverse
    }

    private BiFunction<CANSparkMax, Type, SparkLimitSwitch> getSwitch;
    private Type switchType;
    private boolean enable;

    public LimitSwitchConfiguration(LimitSwitchDirection direction, Type switchType, boolean enable) {
        if (direction == LimitSwitchDirection.kForward) {
            getSwitch = (a, b) -> a.getForwardLimitSwitch(switchType);
        } else if (direction == LimitSwitchDirection.kReverse) {
            getSwitch = (a, b) -> a.getReverseLimitSwitch(switchType);
        }
        this.switchType = switchType;
        this.enable = enable;
    }

    public void apply(CANSparkMax spark) {
        getSwitch.apply(spark, switchType).enableLimitSwitch(enable);
    }

    public boolean differentFrom(CANSparkMax spark) {
        return getSwitch.apply(spark, switchType).isLimitSwitchEnabled() != enable;
    }
}