package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TargetingConstants;

public class TargetingIOSim implements TargetingIO {
    // private DCMotorSim leftTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
    //     50, 0.00019125);
    private DCMotorSim rightTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);
    private DCMotorSim extensionMotorSimulated = new DCMotorSim(DCMotor.getNEO(1),
        50, 0.00019125);

    // private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;
    private double extensionMotorVoltage = 0.0;
    private double cappedExtensionSpeed;

    // private double leftPosition;
    private double rightPosition;
    private double extensionPosition;

    private boolean anglerLimitsOff = false;
    private boolean extensionLimitsOff = false;


    public TargetingIOSim() {

    }

    @Override
    public void setTargetingSpeedPercent(double speed) {  // TODO negative or positive limits & speeds
        double speedClamped = speed;
        speedClamped = clampSpeeds(rightPosition, speedClamped);
        rightTargetingMotorSimulated.setInputVoltage(speedClamped * 12);
        System.out.println("Speed " + speedClamped);
        rightMotorVoltage = speedClamped * 12;
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // leftMotorVoltage = voltage;
        rightMotorVoltage = voltage;
        // leftTargetingMotorSimulated.setInputVoltage(voltage);
        rightTargetingMotorSimulated.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {

        // leftTargetingMotorSimulated.update(0.02);
        rightTargetingMotorSimulated.update(0.02);
        extensionMotorSimulated.update(0.02);

        // inputs.leftTargetingSpeedPercent = leftMotorVoltage/12;
        // inputs.leftTargetingAppliedVoltage = leftMotorVoltage;
        // inputs.leftTargetingCurrentAmps = leftTargetingMotorSimulated.getCurrentDrawAmps();
        // inputs.leftTargetingPositionDegrees += leftTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        // leftPositon = inputs.leftTargetingPositionDegrees;

        inputs.rightTargetingSpeedPercent = rightMotorVoltage/12;
        inputs.rightTargetingAppliedVoltage = rightMotorVoltage;
        inputs.rightTargetingCurrentAmps = rightTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.rightTargetingPositionDegrees += rightTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        rightPosition = inputs.rightTargetingPositionDegrees;

        inputs.targetingPositionAverage = getPositionAverage(rightPosition, rightPosition);

        inputs.extensionSpeedPercent = extensionMotorVoltage/12;
        inputs.extensionAppliedVoltage = extensionMotorVoltage;
        inputs.extensionCurrentAmps = extensionMotorSimulated.getCurrentDrawAmps();
        inputs.extensionPosition += ((extensionMotorSimulated.getAngularVelocityRPM()) / 60 * 0.02) * 4; // TODO gears
        extensionPosition = inputs.extensionPosition;
    }

    /**
     * Converts the value of the encoder to a measurement in degrees.
     * 
     * @param motorEncoderValue the encoder value to convert.
     * @return The position of the encoder in degrees.
     */
    public double encoderToDegrees(double motorEncoderValue) { // TODO conversion
        return motorEncoderValue;
    }

    @Override
    public void setExtensionPercentOutput(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeedsExtension(extensionPosition, speedClamped);
        extensionMotorSimulated.setInputVoltage(speedClamped * 12);
        cappedExtensionSpeed = speedClamped;
        extensionMotorVoltage = speedClamped * 12;
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        extensionMotorVoltage = voltage;
        extensionMotorSimulated.setInputVoltage(voltage);
    }

    @Override
    public double getExtensionSpeedPercent() {
        return extensionMotorVoltage / 12;
    }

    @Override
    public double getAnglerSpeedPercent() {
        System.out.println("Angler motor voltage reads " + rightMotorVoltage);
        return rightMotorVoltage / 12;
    }

    @Override
    public double getAnglerEncoderValue() {
        return rightPosition;
    }

    @Override
    public double getExtensionEncoderValue() {
        return extensionPosition;
    }

    @Override
    public void toggleAnglerLimits() {
        anglerLimitsOff = !anglerLimitsOff;
    }

    @Override
    public void toggleExtensionLimits() {
        extensionLimitsOff = !extensionLimitsOff;
    }

    @Override
    public boolean getAnglerLimitsOff() {
        return anglerLimitsOff;
    }

    @Override
    public boolean getExtensionLimitsOff() {
        return extensionLimitsOff;
    }

    @Override
    public double clampSpeeds(double pos, double speed) {
        if (anglerLimitsOff == false) {
            double speedClamped = speed;
            if (pos < TargetingConstants.angleLowerLimit) {
                speedClamped = Math.max(speed, 0);
            }
            if (pos > TargetingConstants.angleUpperLimit) {
                speedClamped = Math.min(speed, 0);
            }
            return speedClamped;
        }
        else {
            return speed;
        }
    }

    @Override
    public double clampSpeedsExtension(double pos, double speed) {
        if (extensionLimitsOff == false) {
            double speedClamped = speed;
            if (pos < TargetingConstants.extensionLowerLimit) {
                speedClamped = Math.max(speed, 0);
            }
            if (pos > TargetingConstants.extensionUpperLimit) {
                speedClamped = Math.min(speed, 0);
            }
            return speedClamped;
        }
        else {
            return speed;
        }
    }
}
