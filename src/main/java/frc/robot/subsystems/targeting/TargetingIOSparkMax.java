package frc.robot.subsystems.targeting;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Resolver;

public class TargetingIOSparkMax implements TargetingIO {
    private final CANSparkMax leftTargetingMotor;
    private final CANSparkMax rightTargetingMotor;
    private final CANSparkMax extensionMotor;
    private final Resolver targetingEncoder = new Resolver(7, TargetingConstants.targetingMinVoltage,
            TargetingConstants.targetingMaxVoltage, 0, false);

    // TODO override voltage methods

    public TargetingIOSparkMax() {
        leftTargetingMotor = new CANSparkMax(TargetingConstants.leftAnglerCanID, MotorType.kBrushless);
        rightTargetingMotor = new CANSparkMax(TargetingConstants.rightAnglerCanID, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(TargetingConstants.extensionCanID, MotorType.kBrushless);
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
        inputs.leftTargetingPositionDegrees = targetingEncoder.getD();

        inputs.rightTargetingSpeedPercent = rightTargetingMotor.getAppliedOutput();
        inputs.rightTargetingAppliedVoltage = rightTargetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightTargetingCurrentAmps = rightTargetingMotor.getOutputCurrent();
        inputs.rightTargetingTempCelsius = rightTargetingMotor.getMotorTemperature();
        inputs.rightTargetingPositionDegrees = targetingEncoder.getD();

        inputs.targetingPositionAverage = targetingEncoder.getD();

        inputs.extensionSpeedPercent = extensionMotor.getAppliedOutput();
        inputs.extensionAppliedVoltage = extensionMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.extensionCurrentAmps = extensionMotor.getOutputCurrent();
        inputs.extensionTempCelsius = extensionMotor.getMotorTemperature();
        inputs.extensionPosition = extensionMotor.getEncoder().getPosition(); // TODO set
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
    public void setExtensionPercentOutput(double speed) {  // TODO negative or positive limits & speeds
        double speedClamped = speed;
        speedClamped = clampSpeedsExtension(extensionMotor.getEncoder().getPosition(), speedClamped); // TODO encoder technicalities
        extensionMotor.set(speedClamped);
    }

    @Override
    public double getExtensionPosition() {
        return 0; // TODO return
    }
}
