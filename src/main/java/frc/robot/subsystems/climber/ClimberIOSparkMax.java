package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public ClimberIOSparkMax() {
        leftMotor = new CANSparkMax(ClimberConstants.leftCanID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimberConstants.rightCanID, MotorType.kBrushless);
    }

    @Override
    public void setLeftVoltage(double lVoltage){
        leftMotor.setVoltage(lVoltage);
    }

    @Override
    public void setRightVoltage(double rVoltage){
        rightMotor.setVoltage(rVoltage);
    }

    @Override
    public void setLeftSpeedPercent(double lSpeed){
        leftMotor.set(lSpeed);
    }

    @Override
    public void setRightSpeedPercent(double rSpeed){
        leftMotor.set(rSpeed);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.leftSpeedPercent = leftMotor.getAppliedOutput();
        inputs.leftAppliedVoltage = leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.leftTempCelcius = leftMotor.getMotorTemperature();

        inputs.rightSpeedPercent = rightMotor.getAppliedOutput();
        inputs.rightAppliedVoltage = rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.rightTempCelcius = rightMotor.getMotorTemperature();
    }
}
