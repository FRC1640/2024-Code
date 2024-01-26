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
    public void setTargetingSpeedPercent(double speed) {
        leftTargetingMotor.set(speed);
        rightTargetingMotor.set(speed);
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        leftTargetingMotor.setVoltage(voltage);
        rightTargetingMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {
        inputs.leftTargetingSpeedPercent = leftTargetingMotor.get();
        inputs.leftTargetingAppliedVoltage = leftTargetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftTargetingCurrentAmps = leftTargetingMotor.getOutputCurrent();
        inputs.leftTargetingTempCelsius = leftTargetingMotor.getMotorTemperature();
        inputs.leftTargetingPosition = leftTargetingMotor.getEncoder().getPosition();

        inputs.rightTargetingSpeedPercent = rightTargetingMotor.get();
        inputs.rightTargetingAppliedVoltage = rightTargetingMotor.getAppliedOutput();
        inputs.rightTargetingCurrentAmps = rightTargetingMotor.getOutputCurrent();
        inputs.rightTargetingTempCelsius = rightTargetingMotor.getMotorTemperature();
        inputs.rightTargetingPosition = rightTargetingMotor.getEncoder().getPosition();

    }
}
