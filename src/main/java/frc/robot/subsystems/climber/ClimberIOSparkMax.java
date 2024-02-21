package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.sensors.Resolver;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final Resolver leftEncoder;
    private final Resolver rightEncoder;

    public ClimberIOSparkMax() {
        leftMotor = new CANSparkMax(ClimberConstants.leftCanID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.rightCanID, MotorType.kBrushless);
        leftEncoder = new Resolver(ClimberConstants.leftClimberResolver, .05, 4.95, 0, false);
        rightEncoder = new Resolver(ClimberConstants.rightClimberResolver, .05, 4.95, 0, false); //note: to create resolver constants class for min/max
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

        inputs.rightSpeedPercent = rightMotor.getAppliedOutput();
        inputs.rightAppliedVoltage = rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.rightTempCelcius = rightMotor.getMotorTemperature();
        inputs.rightClimberPositionDegrees = rightEncoder.getD();
    }
}
