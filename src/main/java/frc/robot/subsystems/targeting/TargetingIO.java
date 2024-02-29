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

        public double targetingPositionAverage = 0.0;

        public double extensionSpeedPercent = 0.0;
        public double extensionAppliedVoltage = 0.0;
        public double extensionCurrentAmps = 0.0;
        public double extensionTempCelsius = 0.0;
        public double extensionPosition = 0.0;
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
     * @param pos the current position.
     * @param speed the base speed to clamp.
     * @return clamped speed.
     */
    public default double clampSpeeds(double pos, double speed) {
        double speedClamped = speed;
        if (pos < TargetingConstants.angleLowerLimit) {
            speedClamped = Math.max(speed, 0);
        }
        if (pos > TargetingConstants.angleUpperLimit) {
            speedClamped = Math.min(speed, 0);
        }
        return speedClamped;
    }

    /**
     * Sets the voltage of the extension motor.
     *  
     * @param voltage the voltage to set the motor to.
     */
    public default void setExtensionVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        setExtensionPercentOutput(voltage / 12);
    }

    /**
     * Sets the percent output of the extension motor.
     * 
     * @param percentOutput the percent output to set the motor to.
     */
    public default void setExtensionPercentOutput(double percentOutput) {
    }

    /**
     * Modifies the inputted speed so as to not move out of extension limits.
     * 
     * @param pos the current position.
     * @param speed the base speed to cap.
     * @return Capped speed.
     */
    public default double clampSpeedsExtension(double pos, double speed) {
        double speedClamped = speed;
        if (pos < TargetingConstants.extensionLowerLimit) {
            speedClamped = Math.max(speed, 0);
        }
        if (pos > TargetingConstants.extensionUpperLimit) {
            speedClamped = Math.min(speed, 0);
        }
        return speedClamped;
    }

    /**
     * Sets the encoder value to 0.
     */
    public default void resetEncoderValue() {
    }

    /**
     * Gets the position of the extension.
     * 
     * @return Position of the extension.
     */
    public default double getExtensionPosition() {
        return 0;
    }

    public default double getCappedExtensionSpeed() {
        return 0;
    }

    public default double getAnglerSpeedPercent() {
        System.out.println("Default called. Something is wrong.");
        return 0;
    }
    
    public default double getExtensionSpeedPercent() {
        return 0;
    }

    public default double getAnglerEncoderValue() {
        return 0;
    }

    public default double getExtensionEncoderValue() {
        return 0;
    }

    public default void toggleAnglerLimits() {
    }

    public default void toggleExtensionLimits() {
    }

    public default boolean getAnglerLimitsOff() {
        return false;
    }

    public default boolean getExtensionLimitsOff() {
        return false;
    }
}