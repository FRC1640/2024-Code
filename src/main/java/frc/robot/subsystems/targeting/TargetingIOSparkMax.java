package frc.robot.subsystems.targeting;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.TargetingConstants;
import frc.robot.sensors.Resolvers.ResolverPointSlope;

public class TargetingIOSparkMax implements TargetingIO {
    private final SparkMax targetingMotor;
    private final ResolverPointSlope targetingEncoder;
    private final PWM blower;

    public TargetingIOSparkMax() {
        targetingMotor = TargetingConstants.getAnglerSpark(TargetingConstants.rightAngleMotorId);
        targetingEncoder = new ResolverPointSlope(TargetingConstants.resolverID, 1.375, 2.0703, 28, 64);
        blower = new PWM(9);
    }

    @Override
    public void setTargetingSpeedPercent(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeeds(targetingEncoder.getD(), speedClamped);
        targetingMotor.set(speedClamped);
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        double speedClamped = voltage;
        speedClamped = clampSpeeds(targetingEncoder.getD(), speedClamped);
        targetingMotor.setVoltage(speedClamped);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {
        inputs.rightTargetingSpeedPercent = targetingMotor.getAppliedOutput();
        inputs.rightTargetingAppliedVoltage = targetingMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightTargetingCurrentAmps = targetingMotor.getOutputCurrent();
        inputs.rightTargetingTempCelsius = targetingMotor.getMotorTemperature();
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
