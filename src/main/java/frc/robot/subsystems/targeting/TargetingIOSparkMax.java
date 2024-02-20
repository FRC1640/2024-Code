package frc.robot.subsystems.targeting;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.ResolverSlope;

public class TargetingIOSparkMax implements TargetingIO {
    // private final CANSparkMax leftTargetingMotor;
    private final CANSparkMax rightTargetingMotor;
    private final CANSparkMax extensionMotor;
    private final ResolverSlope targetingEncoder = new ResolverSlope(TargetingConstants.resolverID, 0.52, 0,
        59, 30, 30);

    public TargetingIOSparkMax() {
        // leftTargetingMotor = new CANSparkMax(TargetingConstants.leftAngleMotorId, MotorType.kBrushless);
        rightTargetingMotor = new CANSparkMax(TargetingConstants.rightAngleMotorId, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(TargetingConstants.extensionMotorId, MotorType.kBrushless);
        rightTargetingMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setTargetingSpeedPercent(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeeds(targetingEncoder.getD(), speedClamped);
        // leftTargetingMotor.set(speedClamped);
        rightTargetingMotor.set(speedClamped);
    }

    // @Override
    // public void setTargetingVoltage(double voltage) {
    //     leftTargetingMotor.setVoltage(voltage);
    //     rightTargetingMotor.setVoltage(voltage);
    // }

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
        inputs.targetingVoltage = targetingEncoder.getV();
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
        speedClamped = clampSpeedsExtension(extensionMotor.getEncoder().getPosition(), speedClamped);
        extensionMotor.set(speedClamped);
    }
}
