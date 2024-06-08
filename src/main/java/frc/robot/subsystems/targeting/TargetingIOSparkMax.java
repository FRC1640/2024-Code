package frc.robot.subsystems.targeting;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Resolvers.ResolverPointSlope;

public class TargetingIOSparkMax implements TargetingIO {
    // private final CANSparkMax leftTargetingMotor;
    private final CANSparkMax rightTargetingMotor;
   
    private final ResolverPointSlope targetingEncoder = new ResolverPointSlope(TargetingConstants.resolverID, 1.375,2.0703,28,64);

    PWM blower = new PWM(9);

    public TargetingIOSparkMax() {
        // leftTargetingMotor = new CANSparkMax(TargetingConstants.leftAngleMotorId, MotorType.kBrushless);
        rightTargetingMotor = new CANSparkMax(TargetingConstants.rightAngleMotorId, MotorType.kBrushless);
        rightTargetingMotor.setIdleMode(IdleMode.kBrake);
        rightTargetingMotor.setInverted(true);
        Constants.updateStatusFrames(rightTargetingMotor, 100, 20, 20, 500, 500, 500, 500);
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
        double speedClamped = voltage;
        speedClamped = clampSpeeds(targetingEncoder.getD(), speedClamped);
        rightTargetingMotor.setVoltage(speedClamped);
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
        inputs.rightTargetingPositionDegrees = targetingEncoder.getD() + 1.5;
        // inputs.rightRadiansPerSecond = rightTargetingMotor.getEncoder().getVelocity() / 60 * 2 * Math.PI / 100;
        inputs.rightRadiansPerSecond = targetingEncoder.getVelocityRadians();

        inputs.targetingPositionAverage = targetingEncoder.getD() + 1.5;
    }
    @Override
    public void runBlower(double speed){
        blower.setSpeed(speed);
    }
}
