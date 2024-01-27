package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;

public interface TargetingIO {
    @AutoLog
    public static class TargetingIOInputs {
        public double leftTargetingSpeedPercent = 0.0;
        public double leftTargetingAppliedVoltage = 0.0;
        public double leftTargetingCurrentAmps = 0.0;
        public double leftTargetingTempCelsius = 0.0;
        public double leftTargetingPositionDegrees = 0.0;

        public double rightTargetingSpeedPercent = 0.0;
        public double rightTargetingAppliedVoltage = 0.0;
        public double rightTargetingCurrentAmps = 0.0;
        public double rightTargetingTempCelsius = 0.0;
        public double rightTargetingPositionDegrees = 0.0;

        public double targetingPositionAverage = 0.0;
    }

    public default void updateInputs(TargetingIOInputs inputs) {
    }

    public default void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        setTargetingSpeedPercent(voltage/12);
    }

    public default void setTargetingSpeedPercent(double speed) {
    }

    public default double getPositionAverage(double motorOneEncoderValue, double motorTwoEncoderValue) {
        return (motorOneEncoderValue + motorTwoEncoderValue)/2;
    }
}