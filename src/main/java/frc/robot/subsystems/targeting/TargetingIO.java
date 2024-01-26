package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.AutoLog;

public interface TargetingIO {
    @AutoLog
    public static class TargetingIOInputs {
        public double leftTargetingSpeedPercent = 0.0;
        public double leftTargetingAppliedVoltage = 0.0;
        public double leftTargetingCurrentAmps = 0.0;
        public double leftTargetingTempCelsius = 0.0;
        public double leftTargetingPosition = 0.0;

        public double rightTargetingSpeedPercent = 0.0;
        public double rightTargetingAppliedVoltage = 0.0;
        public double rightTargetingCurrentAmps = 0.0;
        public double rightTargetingTempCelsius = 0.0;
        public double rightTargetingPosition = 0.0;

    }

    public default void updateInputs(TargetingIOInputs inputs) {
    }

    public default void setTargetingVoltage(double voltage) {
    }

    public default void setTargetingSpeedPercent(double speed) {
    }
}