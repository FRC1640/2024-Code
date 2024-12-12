package frc.robot.subsystems.extension;


import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.TargetingConstants;
import frc.robot.util.motor.SparkMaxConfigurer;

public class ExtensionIOSparkMax implements ExtensionIO{
    private final SparkMax extensionMotor;
    public ExtensionIOSparkMax(){
        extensionMotor = SparkMaxConfigurer.configSpark(
                TargetingConstants.extensionMotorId, TargetingConstants.sparkDefaultsExtension); //TODO: Jake
    }

    @Override
    public void updateInputs(ExtensionIOInputs inputs){
        
        inputs.extensionSpeedPercent = extensionMotor.getAppliedOutput();
        inputs.extensionAppliedVoltage = extensionMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.extensionCurrentAmps = extensionMotor.getOutputCurrent();
        inputs.extensionTempCelsius = extensionMotor.getMotorTemperature();
        inputs.extensionPosition = extensionMotor.getEncoder().getPosition();
        inputs.extensionLimitSwitch = extensionMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed(); //TODO something changed here with REV.
    }

    
    @Override
    public void setExtensionPercentOutput(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeedsExtensionPercent(extensionMotor.getEncoder().getPosition(), speedClamped);
        extensionMotor.set(speedClamped);
    }
    @Override
    public void setExtensionVoltage(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeedsExtensionVoltage(extensionMotor.getEncoder().getPosition(), speedClamped);
        extensionMotor.setVoltage(speedClamped);
    }

    @Override
    public void resetExtension(){
        extensionMotor.getEncoder().setPosition(0);
    }
}
