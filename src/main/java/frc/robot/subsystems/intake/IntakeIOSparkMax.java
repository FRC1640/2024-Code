package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax intakeMotor;

    public IntakeIOSparkMax() {
        intakeMotor = new CANSparkMax(-3, MotorType.kBrushless); // TODO ids
    }

    @Override
    public void setSpeedPercent(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSpeedPercent = intakeMotor.get();
        inputs.intakeAppliedVoltage = intakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeTempCelsius = intakeMotor.getMotorTemperature();
        inputs.hasNote = false; // TODO sensing
    }

}
