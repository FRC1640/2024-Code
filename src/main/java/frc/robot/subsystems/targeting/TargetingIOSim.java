package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TargetingIOSim implements TargetingIO {
    private DCMotorSim leftTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);
    private DCMotorSim rightTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);

    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;

    private double leftPositon;
    private double rightPosition;



    public TargetingIOSim() {

    }

    @Override
    public void setTargetingSpeedPercent(double speed) {
        double speedClamped = speed;
        // double averagePosition = getPositionAverage(leftPositon, rightPosition);
        speedClamped = clampSpeeds(rightPosition, speedClamped);
        setTargetingVoltage(speedClamped * 12);

    }

    @Override 
    public double getPositionAverage(double motorOneEncoderValue, double motorTwoEncoderValue) {
        return motorTwoEncoderValue;
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // leftMotorVoltage = voltage;
        rightMotorVoltage = voltage;
        double speedClamped = voltage;
        // double averagePosition = getPositionAverage(leftPositon, rightPosition);
        speedClamped = clampSpeeds(rightPosition, speedClamped);
        // leftTargetingMotorSimulated.setInputVoltage(voltage);
        rightTargetingMotorSimulated.setInputVoltage(speedClamped);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {

        leftTargetingMotorSimulated.update(0.02);
        rightTargetingMotorSimulated.update(0.02);

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

        inputs.targetingPositionAverage = getPositionAverage(leftPositon, rightPosition);
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
}
