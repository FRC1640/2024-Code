package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TargetingConstants;

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

    /**
     * Updates the AdvantageKit inputs.
     * 
     * @param inputs the TargetingIOInputs to update.
     */
    public default void updateInputs(TargetingIOInputs inputs) {
    }

    /**
     * Sets the voltage of the motors.
     *  
     * @param voltage the voltage to set the motor to.
     */
    public default void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        setTargetingSpeedPercent(voltage / 12);
    }

    /**
     * Sets the speed of the motors as a percent.
     * 
     * @param percentOutput the percent output to set the motor to.
     */
    public default void setTargetingSpeedPercent(double percentOutput) {
    }

    /**
     * Returns the average of two encoder values.
     * 
     * @param motorOneEncoderValue the first encoder value.
     * @param motorTwoEncoderValue the second encoder value.
     * @return The average of the values.
     */
    public default double getPositionAverage(double motorOneEncoderValue, double motorTwoEncoderValue) {
        return (motorOneEncoderValue + motorTwoEncoderValue) / 2;
    }

    /**
     * Modifies the inputted speed so as to not move out of limits, applying a deadband as well.
     * 
     * @param pos the current position.
     * @param speed the base speed to cap.
     * @return Capped speed.
     */
    public default double clampSpeeds(double pos, double speed) {
        double speedClamped = speed;
        if (pos < TargetingConstants.targetingLowerLimit) {
            speedClamped = Math.max(speed, 0);
        }
        if (pos > TargetingConstants.targetingUpperLimit) {
            speedClamped = Math.min(speed, 0);
        }
        return speedClamped;
    }
}