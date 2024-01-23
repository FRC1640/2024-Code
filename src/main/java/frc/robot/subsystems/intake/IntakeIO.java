package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeSpeedPercent = 0.0;
        public double intakeAppliedVoltage = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeTempCelsius = 0.0;

        public double indexerSpeedPercent = 0.0;
        public double indexerAppliedVoltage = 0.0;
        public double indexerCurrentAmps = 0.0;
        public double indexerTempCelsius = 0.0;

        public boolean hasNote = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakeVoltage(double voltage) {
    }

    public default void setIntakeSpeedPercent(double speed) {
    }

    public default void setIndexerVoltage(double voltage) {
    }

    public default void setIndexerSpeedPercent(double speed) {
    }

}
