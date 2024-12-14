package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class IntakeIOSim implements IntakeIO {
    LinearSystem<N2,N1,N2> plant = LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .00019125, 50);
    private DCMotorSim intakeMotor = new DCMotorSim(plant, DCMotor.getNEO(1), 0);
    private DCMotorSim indexerMotor = new DCMotorSim(plant, DCMotor.getNEO(1), 0);

    private double indexerMotorVoltage = 0.0;
    private double intakeMotorVoltage = 0.0;
    BooleanSupplier note;

    public IntakeIOSim(BooleanSupplier note){
        this.note = note;
    }
    @Override
    public void setIndexerSpeedPercent(double speed) {
        setIndexerVoltage(speed * 12);
    }
    @Override
    public void setIndexerVoltage(double voltage) {
        indexerMotorVoltage = voltage;
        indexerMotor.setInputVoltage(voltage);
    }
    @Override
    public void setIntakeSpeedPercent(double speed) {
        setIntakeVoltage(speed * 12);
    }
    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotorVoltage = voltage;
        intakeMotor.setInputVoltage(voltage);
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSpeedPercent = intakeMotorVoltage / 12;
        inputs.intakeAppliedVoltage = intakeMotorVoltage;
        inputs.intakeCurrentAmps = intakeMotor.getCurrentDrawAmps();

        inputs.indexerSpeedPercent = indexerMotorVoltage / 12;
        inputs.indexerAppliedVoltage = indexerMotorVoltage;
        inputs.indexerCurrentAmps = indexerMotor.getCurrentDrawAmps();
        
        inputs.hasNote = note.getAsBoolean();
    }
}
