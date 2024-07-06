package frc.robot.subsystems.intake;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SparkMaxDefaults;
import frc.robot.util.motor.SparkMaxConfiguration;
import frc.robot.util.motor.SparkMaxConfigurer;
import frc.robot.util.motor.StatusFrames;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax indexerMotor;
    private DigitalInput proximityDigitalInput;
    BooleanSupplier hasNote;

    long initTime;
    boolean first = false;
    private BooleanSupplier clearCondition;

    private boolean autoStarted;
    private BooleanSupplier startAuto;
    

    public IntakeIOSparkMax(BooleanSupplier hasNote, BooleanSupplier clearCondition, BooleanSupplier startAuto) {
        this.clearCondition = clearCondition;
        this.startAuto = startAuto;
        intakeMotor = SparkMaxConfigurer.configSpark(
                IntakeConstants.intakeCanID,
                new SparkMaxConfiguration(
                    IdleMode.kBrake,
                    true,
                    SparkMaxDefaults.limitSwitch,
                    SparkMaxDefaults.limSwitchType,
                    SparkMaxDefaults.smartCurrentLimit,
                    SparkMaxDefaults.encoderMeasurementPeriod,
                    SparkMaxDefaults.encoderAverageDepth,
                    SparkMaxDefaults.canTimeout,
                    new StatusFrames(100, 200, 200,
                        500, 500, 500, 500),
                        "Intake/Intake flashed?"));
        indexerMotor = SparkMaxConfigurer.configSpark(
                IntakeConstants.indexerCanID,
                new SparkMaxConfiguration(
                    IdleMode.kBrake,
                    true,
                    SparkMaxDefaults.limitSwitch,
                    SparkMaxDefaults.limSwitchType,
                    SparkMaxDefaults.smartCurrentLimit,
                    SparkMaxDefaults.encoderMeasurementPeriod,
                    SparkMaxDefaults.encoderAverageDepth,
                    SparkMaxDefaults.canTimeout,
                    new StatusFrames(100, 200, 200,
                        500, 500, 500, 500),
                    "Intake/Indexer flashed?"));
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
        // if (autoStarted){
        //     autoStarted = !autoStarted;
        // }
        // if (startAuto.getAsBoolean()){
        //     autoStarted = true;
        // }
        // inputs.hasNote = noteDelay(!proximityDigitalInput.get() || autoStarted, clearCondition.getAsBoolean());
        inputs.hasNote = !proximityDigitalInput.get();


    } 

    public boolean noteDelay(boolean value, boolean clear){
        if (value){
            first = true;
        }
        if (clear){
            first = false;
        }
        return first;
    }
}
