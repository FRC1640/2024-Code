package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TargetingIOSim implements TargetingIO {
    private DCMotorSim leftTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);
    private DCMotorSim rightTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);
    private DCMotorSim extensionMotorSimulated = new DCMotorSim(DCMotor.getNEO(1),
        50, 0.00019125);
    
    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;
    private double extensionMotorVoltage = 0.0;

    private double leftPositon;
    private double rightPosition;
    private double extensionPosition;
    private double cappedExtensionSpeed;


    public TargetingIOSim() {

    }

    @Override
    public void setTargetingPercentOutput(double speed) {  // TODO negative or positive limits & speeds
        double speedClamped = speed;
        double averagePosition = getPositionAverage(leftPositon, rightPosition);
        speedClamped = clampSpeedsTargeting(averagePosition, speedClamped);
        setTargetingVoltage(speedClamped * 12);

    }

    @Override
    public void setExtensionPercentOutput(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeedsExtension(extensionPosition, speedClamped);
        setExtensionVoltage(speedClamped * 12);
        cappedExtensionSpeed = speedClamped;
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        leftMotorVoltage = voltage;
        rightMotorVoltage = voltage;
        leftTargetingMotorSimulated.setInputVoltage(voltage);
        rightTargetingMotorSimulated.setInputVoltage(voltage);
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        extensionMotorVoltage = voltage;
        extensionMotorSimulated.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {

        leftTargetingMotorSimulated.update(0.02);
        rightTargetingMotorSimulated.update(0.02);
        extensionMotorSimulated.update(0.02);

        inputs.leftTargetingSpeedPercent = leftMotorVoltage/12;
        inputs.leftTargetingAppliedVoltage = leftMotorVoltage;
        inputs.leftTargetingCurrentAmps = leftTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.leftTargetingPositionDegrees += leftTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        leftPositon = inputs.leftTargetingPositionDegrees;

        inputs.rightTargetingSpeedPercent = rightMotorVoltage/12;
        inputs.rightTargetingAppliedVoltage = rightMotorVoltage;
        inputs.rightTargetingCurrentAmps = rightTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.rightTargetingPositionDegrees += rightTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        rightPosition = inputs.rightTargetingPositionDegrees;

        inputs.targetingPositionAverage = getPositionAverage(leftPositon, rightPosition);

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
    public double getExtensionPosition() {
        return extensionPosition;
    }

    @Override
    public double getCappedExtensionSpeed() {
        return cappedExtensionSpeed;
    }
}
