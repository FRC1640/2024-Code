package frc.robot.subsystems.targeting;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.TargetingConstants;

public class TargetingIOSparkMax implements TargetingIO {
    private final CANSparkMax leftTargetingMotor;
    private final CANSparkMax rightTargetingMotor;

    public TargetingIOSparkMax() {
        leftTargetingMotor = new CANSparkMax(TargetingConstants.leftTargetingMotorId, MotorType.kBrushless);
        rightTargetingMotor = new CANSparkMax(TargetingConstants.rightTargetingMotorId, MotorType.kBrushless);
    }

    @Override
    public void setTargetingSpeedPercent(double speed) {  // TODO negative or positive limits & speeds
        double speedClamped = speed;
        double averagePosition = getPositionAverage(leftTargetingMotor.getEncoder().getPosition(),
                rightTargetingMotor.getEncoder().getPosition());
        averagePosition = encoderToDegrees(averagePosition);
        speedClamped = clampSpeeds(averagePosition, speedClamped);
        leftTargetingMotor.set(speedClamped);
        rightTargetingMotor.set(speedClamped);        
    }

    // @Override
    // public void setTargetingVoltage(double voltage) {
    //     leftTargetingMotor.setVoltage(voltage);
    //     rightTargetingMotor.setVoltage(voltage);
    // }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {
        inputs.leftTargetingSpeedPercent = leftTargetingMotor.getAppliedOutput();
        inputs.leftTargetingAppliedVoltage = leftTargetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftTargetingCurrentAmps = leftTargetingMotor.getOutputCurrent();
        inputs.leftTargetingTempCelsius = leftTargetingMotor.getMotorTemperature();
        inputs.leftTargetingPositionDegrees = encoderToDegrees(leftTargetingMotor.getEncoder().getPosition());

        inputs.rightTargetingSpeedPercent = rightTargetingMotor.getAppliedOutput();
        inputs.rightTargetingAppliedVoltage = rightTargetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightTargetingCurrentAmps = rightTargetingMotor.getOutputCurrent();
        inputs.rightTargetingTempCelsius = rightTargetingMotor.getMotorTemperature();
        inputs.rightTargetingPositionDegrees = encoderToDegrees(rightTargetingMotor.getEncoder().getPosition());

        inputs.targetingPositionAverage = getPositionAverage(encoderToDegrees(leftTargetingMotor.getEncoder().getPosition()),
                encoderToDegrees(rightTargetingMotor.getEncoder().getPosition()));
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
