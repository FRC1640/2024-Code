package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {
    private final SparkMax topLeftShooter, bottomLeftShooter,topRightShooter, bottomRightShooter;

    public ShooterIOSparkMax() {
        topLeftShooter = SparkMaxConfigurer.configSpark(ShooterConstants.topLeftCanID, ShooterConstants.getSparkDefaultsShooter(true)); // top left can id, spark defaults shooter from constants, inverted
        bottomLeftShooter = SparkMaxConfigurer.configSpark(ShooterConstants.bottomLeftCanID, ShooterConstants.getSparkDefaultsShooter(false)); // bottom left can id, spark defaults shooter from constants, not inverted
        topRightShooter = SparkMaxConfigurer.configSpark(ShooterConstants.topRightCanID, ShooterConstants.getSparkDefaultsShooter(false)); // top right can id, spark defaults shooter from constants, not inverted
        bottomRightShooter = SparkMaxConfigurer.configSpark(ShooterConstants.bottomRightCanID, ShooterConstants.getSparkDefaultsShooter(true)); // bottom right can id, spark defaults shooter from constants, inverted
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
