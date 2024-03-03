package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topLeftSpeedPercent = 0.0;
        public double topLeftAppliedVoltage = 0.0;
        public double topLeftCurrentAmps = 0.0;
        public double topLeftTempCelsius = 0.0;

        public double bottomLeftSpeedPercent = 0.0;
        public double bottomLeftAppliedVoltage = 0.0;
        public double bottomLeftCurrentAmps = 0.0;
        public double bottomLeftTempCelsius = 0.0;

        public double topRightSpeedPercent = 0.0;
        public double topRightAppliedVoltage = 0.0;
        public double topRightCurrentAmps = 0.0;
        public double topRightTempCelsius = 0.0;

        public double bottomRightSpeedPercent = 0.0;
        public double bottomRightAppliedVoltage = 0.0;
        public double bottomRightCurrentAmps = 0.0;
        public double bottomRightTempCelsius = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
    }

    public default void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
    }

    public default void testTopLeftSpeed(double speed) {
    }

    public default void testTopRightSpeed(double speed) {
    }

    public default void testBottomLeftSpeed(double speed) {
    }

    public default void testBottomRightSpeed(double speed) {
    }

    public default double getTopLeftSpeed() {
        return 0;
    }

    public default double getTopRightSpeed() {
        return 0;
    }

    public default double getBottomLeftSpeed() {
        return 0;
    }

    public default double getBottomRightSpeed() {
        return 0;
    }
}
