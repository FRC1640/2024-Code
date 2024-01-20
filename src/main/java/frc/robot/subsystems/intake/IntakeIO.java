package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeSpeedPercent = 0.0;
        public double intakeAppliedVoltage = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeTempCelsius = 0.0;
        public boolean hasNote = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setSpeedPercent(double speed) {
    }

}
