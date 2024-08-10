package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TargetingConstants;

public interface TargetingIO {
    @AutoLog
    public static class TargetingIOInputs {
        // public double leftTargetingSpeedPercent = 0.0;
        // public double leftTargetingAppliedVoltage = 0.0;
        // public double leftTargetingCurrentAmps = 0.0;
        // public double leftTargetingTempCelsius = 0.0;
        // public double leftTargetingPositionDegrees = 0.0;

        public double rightTargetingSpeedPercent = 0.0;
        public double rightTargetingAppliedVoltage = 0.0;
        public double rightTargetingCurrentAmps = 0.0;
        public double rightTargetingTempCelsius = 0.0;
        public double rightTargetingPositionDegrees = 0.0;
        public double rightRadiansPerSecond = 0.0;

        public double targetingPosition = 0.0;

        public double targetingVoltage = 0;
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
     * Modifies the inputted speed so as to not move out of limits
     * 
     * @param pos   the current position.
     * @param speed the base speed to clamp.
     * @return clamped speed.
     */
    public default double clampSpeeds(double pos, double speed) {
        double speedClamped = speed;

        if (!(Double.isNaN(speedClamped) || Double.isNaN(pos))) {
            if (pos < TargetingConstants.angleLowerLimit) {
                speedClamped = Math.max(speed, 0);
            }
            if (pos > TargetingConstants.angleUpperLimit) {
                speedClamped = Math.min(speed, 0);
            }
        }
        else{
            speedClamped = 0;
        }

        return speedClamped;
    }

    public default void runBlower(double speed){};
}