package frc.robot.subsystems.intake;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.IntakeConstants;
import lombok.val;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax indexerMotor;
    private DigitalInput proximityDigitalInput;
    BooleanSupplier hasNote;

    long initTime;
    boolean first = false;;
    

    public IntakeIOSparkMax(BooleanSupplier hasNote) {
        intakeMotor = new CANSparkMax(IntakeConstants.intakeCanID, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(IntakeConstants.indexerCanID, MotorType.kBrushless);
        indexerMotor.setInverted(true);
        proximityDigitalInput = new DigitalInput(IntakeConstants.proximitySensorChannel);
        this.hasNote = hasNote;
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
        // inputs.hasNote = proximityAnalogOutput.getVoltage() > IntakeConstants.proximityVoltageThreshold;
        inputs.hasNote = noteDelay(false);
        // inputs.hasNote = !proximityDigitalInput.get();
    } 

    public boolean noteDelay(boolean value){
        if (value && !first){
            first = true;
            initTime = System.currentTimeMillis();
        }
        if (!value){
            first = false;
        }
        if (value && initTime + 100 > System.currentTimeMillis()){
            return true;
        }
        return false;
    }
}
