package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax topLeftShooter, bottomLeftShooter,topRightShooter, bottomRightShooter;

    public ShooterIOSparkMax() {
        topLeftShooter = new CANSparkMax(ShooterConstants.topLeftCanID, MotorType.kBrushless); // TODO ids
        bottomLeftShooter = new CANSparkMax(ShooterConstants.bottomLeftCanID, MotorType.kBrushless);
        topRightShooter = new CANSparkMax(ShooterConstants.topRightCanID, MotorType.kBrushless); // TODO ids
        bottomRightShooter = new CANSparkMax(ShooterConstants.bottomRightCanID, MotorType.kBrushless);
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
        topLeftShooter.setVoltage(topLeft);
        bottomLeftShooter.setVoltage(bottomLeft);
        topRightShooter.setVoltage(topRight);
        bottomRightShooter.setVoltage(bottomRight);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topLeftSpeedPercent = topLeftShooter.get();
        inputs.topLeftAppliedVoltage = topLeftShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.topLeftCurrentAmps = topLeftShooter.getOutputCurrent();
        inputs.topLeftTempCelsius = topLeftShooter.getMotorTemperature();

        inputs.bottomLeftSpeedPercent = bottomLeftShooter.get();
        inputs.bottomLeftAppliedVoltage = bottomLeftShooter.getAppliedOutput() * RobotController.getBatteryVoltage();;
        inputs.bottomLeftCurrentAmps = bottomLeftShooter.getOutputCurrent();
        inputs.bottomLeftTempCelsius = bottomLeftShooter.getMotorTemperature();

        inputs.topRightSpeedPercent = topRightShooter.get();
        inputs.topRightAppliedVoltage = topRightShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.topRightCurrentAmps = topRightShooter.getOutputCurrent();
        inputs.topRightTempCelsius = topRightShooter.getMotorTemperature();

        inputs.bottomRightSpeedPercent = bottomRightShooter.get();
        inputs.bottomRightAppliedVoltage = bottomRightShooter.getAppliedOutput() * RobotController.getBatteryVoltage();;
        inputs.bottomRightCurrentAmps = bottomRightShooter.getOutputCurrent();
        inputs.bottomRightTempCelsius = bottomRightShooter.getMotorTemperature();
    }
}
