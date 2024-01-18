package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topSpeedPercent = 0.0;
        public double topAppliedVoltage = 0.0;
        public double topCurrentAmps = 0.0;
        public double topTempCelsius = 0.0;

        public double bottomSpeedPercent = 0.0;
        public double bottomAppliedVoltage = 0.0;
        public double bottomCurrentAmps = 0.0;
        public double bottomTempCelsius = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVoltage(double top, double bottom) {
    }

    public default void setSpeedPercent(double top, double bottom) {
    }
}
