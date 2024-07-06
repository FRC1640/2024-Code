package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkMaxDefaults;
import frc.robot.util.motor.SparkMaxConfiguration;
import frc.robot.util.motor.SparkMaxConfigurer;
import frc.robot.util.motor.StatusFrames;

public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax topLeftShooter, bottomLeftShooter,topRightShooter, bottomRightShooter;

    public ShooterIOSparkMax() {
        topLeftShooter = SparkMaxConfigurer.configSpark(
            ShooterConstants.topLeftCanID,
            new SparkMaxConfiguration(
                SparkMaxDefaults.idleMode,
                true,
                SparkMaxDefaults.limitSwitch,
                SparkMaxDefaults.limSwitchType,
                SparkMaxDefaults.smartCurrentLimit,
                SparkMaxDefaults.encoderMeasurementPeriod,
                SparkMaxDefaults.encoderAverageDepth,
                SparkMaxDefaults.canTimeout,
                    new StatusFrames(100, 200, 200,
                        500, 500, 500, 500),
                "Shooter TL"));
        bottomLeftShooter = SparkMaxConfigurer.configSpark(
            ShooterConstants.bottomLeftCanID,
            new SparkMaxConfiguration(
                SparkMaxDefaults.idleMode,
                SparkMaxDefaults.inverted,
                SparkMaxDefaults.limitSwitch,
                SparkMaxDefaults.limSwitchType,
                SparkMaxDefaults.smartCurrentLimit,
                SparkMaxDefaults.encoderMeasurementPeriod,
                SparkMaxDefaults.encoderAverageDepth,
                SparkMaxDefaults.canTimeout,
                new StatusFrames(100, 200, 200,
                    500, 500, 500, 500),
                "Shooter BL"));
        topRightShooter = SparkMaxConfigurer.configSpark(
            ShooterConstants.topRightCanID,
            new SparkMaxConfiguration(
                SparkMaxDefaults.idleMode,
                SparkMaxDefaults.inverted,
                SparkMaxDefaults.limitSwitch,
                SparkMaxDefaults.limSwitchType,
                SparkMaxDefaults.smartCurrentLimit,
                SparkMaxDefaults.encoderMeasurementPeriod,
                SparkMaxDefaults.encoderAverageDepth,
                SparkMaxDefaults.canTimeout,
                new StatusFrames(100, 200, 200,
                    500, 500, 500, 500),
                "Shooter TR"));
        bottomRightShooter = SparkMaxConfigurer.configSpark(
            ShooterConstants.bottomRightCanID,
            new SparkMaxConfiguration(
                SparkMaxDefaults.idleMode,
                true,
                SparkMaxDefaults.limitSwitch,
                SparkMaxDefaults.limSwitchType,
                SparkMaxDefaults.smartCurrentLimit,
                SparkMaxDefaults.encoderMeasurementPeriod,
                SparkMaxDefaults.encoderAverageDepth,
                SparkMaxDefaults.canTimeout,
                new StatusFrames(100, 200, 200,
                    500, 500, 500, 500),
                "Shooter BR"));
    }

    @Override
    public void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        topLeftShooter.set(topLeft);
        bottomLeftShooter.set(bottomLeft);
        topRightShooter.set(topRight);
        bottomRightShooter.set(bottomRight);
    }

    @Override
    public void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        double topLeftClamped = 0;
        double bottomLeftClamped = 0;
        double topRightClamped = 0;
        double bottomRightClamped = 0;

        topLeftClamped = MathUtil.clamp(topLeft, -12, 12);
        bottomLeftClamped = MathUtil.clamp(bottomLeft, -12, 12);
        topRightClamped = MathUtil.clamp(topRight, -12, 12);
        bottomRightClamped = MathUtil.clamp(bottomRight, -12, 12);

        topLeftShooter.setVoltage(topLeftClamped);
        bottomLeftShooter.setVoltage(bottomLeftClamped);
        topRightShooter.setVoltage(topRightClamped);
        bottomRightShooter.setVoltage(bottomRightClamped);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topLeftSpeedPercent = topLeftShooter.getEncoder().getVelocity() / 5676;
        inputs.topLeftAppliedVoltage = topLeftShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.topLeftCurrentAmps = topLeftShooter.getOutputCurrent();
        inputs.topLeftTempCelsius = topLeftShooter.getMotorTemperature();
        
        inputs.topLeftVelocity = topLeftShooter.getEncoder().getVelocity() / 60 * 2 * Math.PI;
        inputs.topLeftPositionRadians = topLeftShooter.getEncoder().getPosition() * 2 * Math.PI;

        inputs.bottomLeftSpeedPercent = bottomLeftShooter.getEncoder().getVelocity() / 5676;
        inputs.bottomLeftAppliedVoltage = bottomLeftShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.bottomLeftCurrentAmps = bottomLeftShooter.getOutputCurrent();
        inputs.bottomLeftTempCelsius = bottomLeftShooter.getMotorTemperature();
        inputs.bottomLeftVelocity = bottomLeftShooter.getEncoder().getVelocity() / 60 * 2 * Math.PI;

        inputs.topRightSpeedPercent = topRightShooter.getEncoder().getVelocity() / 5676;
        inputs.topRightAppliedVoltage = topRightShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.topRightCurrentAmps = topRightShooter.getOutputCurrent();
        inputs.topRightTempCelsius = topRightShooter.getMotorTemperature();
        inputs.topRightVelocity = topRightShooter.getEncoder().getVelocity() / 60 * 2 * Math.PI;

        inputs.bottomRightSpeedPercent = bottomRightShooter.getEncoder().getVelocity() / 5676;
        inputs.bottomRightAppliedVoltage = bottomRightShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.bottomRightCurrentAmps = bottomRightShooter.getOutputCurrent();
        inputs.bottomRightTempCelsius = bottomRightShooter.getMotorTemperature();
        inputs.bottomRightVelocity = bottomRightShooter.getEncoder().getVelocity() / 60 * 2 * Math.PI;
    }
}
