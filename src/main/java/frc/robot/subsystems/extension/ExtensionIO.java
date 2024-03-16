package frc.robot.subsystems.extension;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.TargetingConstants;

public interface ExtensionIO {
    @AutoLog
    public static class ExtensionIOInputs{
        public double extensionSpeedPercent = 0.0;
        public double extensionAppliedVoltage = 0.0;
        public double extensionCurrentAmps = 0.0;
        public double extensionTempCelsius = 0.0;
        public double extensionPosition = 0.0;
        public boolean extensionLimitSwitch = false;
    }
        /**
     * Updates the AdvantageKit inputs.
     * 
     * @param inputs the TargetingIOInputs to update.
     */
    public default void updateInputs(ExtensionIOInputs inputs) {
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
     * @param pos   the current position.
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

    public default void resetExtension(){}
}
