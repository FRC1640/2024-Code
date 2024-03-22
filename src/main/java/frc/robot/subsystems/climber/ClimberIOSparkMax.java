package frc.robot.subsystems.climber;

import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.sensors.Resolvers.ResolverPointSlope;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final ResolverPointSlope leftEncoder;
    private final ResolverPointSlope rightEncoder;

    public ClimberIOSparkMax() {
        leftMotor = new CANSparkMax(ClimberConstants.leftCanID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.rightCanID, MotorType.kBrushless);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

        Constants.updateStatusFrames(leftMotor, 100, 20, 20, 500, 500, 500, 500);
        Constants.updateStatusFrames(rightMotor, 100, 20, 20, 500, 500, 500, 500);
        leftEncoder = new ResolverPointSlope(ClimberConstants.leftClimberResolver, 1.327, 2.356, 1, 76);
        rightEncoder = new ResolverPointSlope(ClimberConstants.rightClimberResolver, 3.637, 2.585, 11, 84); //note: to create resolver constants class for min/max
    }

    // @Override
    // public void setLeftVoltage(double lVoltage){
    //     leftMotor.setVoltage(lVoltage);
    // }

    // @Override
    // public void setRightVoltage(double rVoltage){
    //     rightMotor.setVoltage(rVoltage);
    // }

    @Override
    public void setLeftSpeedPercent(double speed){
        speed = clampSpeeds(leftEncoder.getD(), speed);
        leftMotor.set(speed);
    }

    @Override
    public void setRightSpeedPercent(double speed){
        speed = clampSpeeds(rightEncoder.getD(), speed);
        rightMotor.set(speed);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.leftSpeedPercent = leftMotor.getAppliedOutput();
        inputs.leftAppliedVoltage = leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.leftTempCelcius = leftMotor.getMotorTemperature();
        inputs.leftClimberPositionDegrees = leftEncoder.getD();
        inputs.leftClimberVoltage = leftEncoder.getV();

        inputs.rightSpeedPercent = rightMotor.getAppliedOutput();
        inputs.rightAppliedVoltage = rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.rightTempCelcius = rightMotor.getMotorTemperature();
        inputs.rightClimberPositionDegrees = rightEncoder.getD();
        inputs.rightClimberVoltage = rightEncoder.getV();
    }
}
