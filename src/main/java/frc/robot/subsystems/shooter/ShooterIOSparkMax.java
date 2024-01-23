package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax topShooter, bottomShooter;

    public ShooterIOSparkMax() {
        topShooter = new CANSparkMax(ShooterConstants.topCanID, MotorType.kBrushless); // TODO ids
        bottomShooter = new CANSparkMax(ShooterConstants.bottomCanID, MotorType.kBrushless);
    }

    @Override
    public void setSpeedPercent(double top, double bottom) {
        topShooter.set(top);
        bottomShooter.set(bottom);
    }

    @Override
    public void setVoltage(double top, double bottom) {
        topShooter.setVoltage(top);
        bottomShooter.setVoltage(bottom);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topSpeedPercent = topShooter.get();
        inputs.topAppliedVoltage = topShooter.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.topCurrentAmps = topShooter.getOutputCurrent();
        inputs.topTempCelsius = topShooter.getMotorTemperature();

        inputs.bottomSpeedPercent = bottomShooter.get();
        inputs.bottomAppliedVoltage = bottomShooter.getAppliedOutput() * RobotController.getBatteryVoltage();;
        inputs.bottomCurrentAmps = bottomShooter.getOutputCurrent();
        inputs.bottomTempCelsius = bottomShooter.getMotorTemperature();
        
    }
}
