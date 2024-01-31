package frc.robot.subsystems.intake;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax indexerMotor;
    private AnalogOutput proximityAnalogOutput;
    

    public IntakeIOSparkMax() {
        intakeMotor = new CANSparkMax(IntakeConstants.intakeCanID, MotorType.kBrushless); // TODO ids
        
        indexerMotor = new CANSparkMax(IntakeConstants.indexerCanID, MotorType.kBrushless);
        proximityAnalogOutput = new AnalogOutput(Constants.IntakeConstants.proximitySensorChannel);

         
    }

    @Override
    public void setIntakeSpeedPercent(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setIndexerSpeedPercent(double speed) {
        indexerMotor.set(speed);
    }

    @Override
    public void setIndexerVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSpeedPercent = intakeMotor.getAppliedOutput();
        inputs.intakeAppliedVoltage = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeTempCelsius = intakeMotor.getMotorTemperature();

        inputs.indexerSpeedPercent = indexerMotor.getAppliedOutput();
        inputs.indexerAppliedVoltage = indexerMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.indexerCurrentAmps = indexerMotor.getOutputCurrent();
        inputs.indexerTempCelsius = indexerMotor.getMotorTemperature();

        
        inputs.hasNote = proximityAnalogOutput.getVoltage() > Constants.IntakeConstants.proximityVoltageThreshold; // TODO sensing
    } 
}
