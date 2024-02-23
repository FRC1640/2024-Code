package frc.robot.subsystems.targeting;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Resolver;

public class TargetingIOSparkMax implements TargetingIO {
    // private final CANSparkMax leftTargetingMotor;
    private final CANSparkMax rightTargetingMotor;
    private final CANSparkMax extensionMotor;
    private final Resolver targetingEncoder = new Resolver(7, TargetingConstants.angleMinVoltage,
            TargetingConstants.angleMaxVoltage, 0, false);

    public TargetingIOSparkMax() {
        // leftTargetingMotor = new CANSparkMax(TargetingConstants.leftAnglerCanID, MotorType.kBrushless);
        rightTargetingMotor = new CANSparkMax(TargetingConstants.rightAnglerCanID, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(TargetingConstants.extensionCanID, MotorType.kBrushless);
        // leftTargetingMotor = new CANSparkMax(TargetingConstants.leftAngleMotorId, MotorType.kBrushless);
        // extensionMotor = new CANSparkMax(TargetingConstants.extensionMotorId, MotorType.kBrushless);
        rightTargetingMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setTargetingSpeedPercent(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeeds(targetingEncoder.getD(), speedClamped);
        // leftTargetingMotor.set(speedClamped);
        rightTargetingMotor.set(speedClamped);
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        // leftTargetingMotor.setVoltage(voltage);
        rightTargetingMotor.setVoltage(voltage);
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        extensionMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {
        // inputs.leftTargetingSpeedPercent = leftTargetingMotor.getAppliedOutput();
        // inputs.leftTargetingAppliedVoltage = leftTargetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        // inputs.leftTargetingCurrentAmps = leftTargetingMotor.getOutputCurrent();
        // inputs.leftTargetingTempCelsius = leftTargetingMotor.getMotorTemperature();
        // inputs.leftTargetingPositionDegrees = targetingEncoder.getD();

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
        inputs.extensionPosition = extensionMotor.getEncoder().getPosition();
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
        speedClamped = clampSpeedsExtension(extensionMotor.getEncoder().getPosition(), speedClamped); // TODO encoder technicalities
        extensionMotor.set(speedClamped);
    }

    @Override
    public double getExtensionPosition() {
        return extensionMotor.getEncoder().getPosition();
    }

    @Override
    public double getExtensionSpeedPercent() {
        return extensionMotor.getAppliedOutput();
    }

    @Override
    public double getAnglerSpeedPercent() {
        return rightTargetingMotor.getAppliedOutput();
    }

    @Override
    public CANSparkMax getRealAnglerMotorTest() {
        return rightTargetingMotor;
    }

    @Override
    public CANSparkMax getRealExtensionMotorTest() {
        return extensionMotor;
    }
}
