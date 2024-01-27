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
        double speedClamped = 0;
        double averagePosition = getPositionAverage(leftTargetingMotor.getEncoder().getPosition(),
                rightTargetingMotor.getEncoder().getPosition());
        if (averagePosition < TargetingConstants.targetingLowerLimit) {
            speedClamped = Math.max(speed, 0);
        }
        if (averagePosition > TargetingConstants.targetingUpperLimit) {
            speedClamped = Math.min(speed, 0);
        }
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
        inputs.leftTargetingPositionDegrees = encoderToDegrees(leftTargetingMotor);

        inputs.rightTargetingSpeedPercent = rightTargetingMotor.getAppliedOutput();
        inputs.rightTargetingAppliedVoltage = rightTargetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightTargetingCurrentAmps = rightTargetingMotor.getOutputCurrent();
        inputs.rightTargetingTempCelsius = rightTargetingMotor.getMotorTemperature();
        inputs.rightTargetingPositionDegrees = encoderToDegrees(rightTargetingMotor);

        inputs.targetingPositionAverage = getPositionAverage(leftTargetingMotor.getEncoder().getPosition(),
                rightTargetingMotor.getEncoder().getPosition());
    }

    public double encoderToDegrees(CANSparkMax motor) { // TODO conversion
        return motor.getEncoder().getPosition();
    }
}
