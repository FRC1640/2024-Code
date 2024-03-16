package frc.robot.subsystems.extension;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.naming.ldap.ExtendedRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.TargetingConstants;

public class ExtensionIOSparkMax implements ExtensionIO{
    private final CANSparkMax extensionMotor;
    public ExtensionIOSparkMax(){
        extensionMotor = new CANSparkMax(TargetingConstants.extensionMotorId, MotorType.kBrushless);
        extensionMotor.setInverted(true);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        Constants.updateStatusFrames(extensionMotor, 100, 20, 20, 500, 500, 500, 500);
        extensionMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(true);
    }

    @Override
    public void updateInputs(ExtensionIOInputs inputs){
        
        inputs.extensionSpeedPercent = extensionMotor.getAppliedOutput();
        inputs.extensionAppliedVoltage = extensionMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.extensionCurrentAmps = extensionMotor.getOutputCurrent();
        inputs.extensionTempCelsius = extensionMotor.getMotorTemperature();
        inputs.extensionPosition = extensionMotor.getEncoder().getPosition();
        inputs.extensionLimitSwitch = extensionMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    
    @Override
    public void setExtensionPercentOutput(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeedsExtension(extensionMotor.getEncoder().getPosition(), speedClamped);
        extensionMotor.set(speedClamped);
    }

    @Override
    public void resetExtension(){
        extensionMotor.getEncoder().setPosition(0);
    }
}
