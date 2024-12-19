package frc.robot.util.motor;

import java.util.function.BiFunction;
import java.util.function.Function;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;


public class LimitSwitchConfiguration {

    public enum LimitSwitchDirection {
        FORWARD((a) -> a.getForwardLimitSwitch()),
        REVERSE((a) -> a.getReverseLimitSwitch());

        public Function<SparkMax, SparkLimitSwitch> getSwitch;

        LimitSwitchDirection(Function<SparkMax, SparkLimitSwitch> getSwitch) {
            this.getSwitch = getSwitch;
        }
    }

    LimitSwitchDirection direction;
    private boolean enable;

    public LimitSwitchConfiguration(LimitSwitchDirection direction, boolean enable) {
        this.direction = direction;
        this.enable = enable;
    }

    public void apply(SparkMax spark) {
        direction.getSwitch.apply(spark).;
    }
}