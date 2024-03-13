package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax topLeftShooter, bottomLeftShooter,topRightShooter, bottomRightShooter;

    public ShooterIOSparkMax() {
        topLeftShooter = new CANSparkMax(ShooterConstants.topLeftCanID, MotorType.kBrushless); 
        bottomLeftShooter = new CANSparkMax(ShooterConstants.bottomLeftCanID, MotorType.kBrushless);
        topRightShooter = new CANSparkMax(ShooterConstants.topRightCanID, MotorType.kBrushless); 
        bottomRightShooter = new CANSparkMax(ShooterConstants.bottomRightCanID, MotorType.kBrushless);


        
        // bottomRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        // bottomRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		// bottomRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		// bottomRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        
        // topRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        // topRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		// topRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		// topRightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);


        
        // bottomLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        // bottomLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		// bottomLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		// bottomLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);


        
        // topLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        // topLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		// topLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		// topLeftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);


        topLeftShooter.setInverted(true);
        bottomRightShooter.setInverted(true);
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
