package frc.robot.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ExtensionIOSim implements ExtensionIO{

    private double extensionMotorVoltage = 0.0;
    LinearSystem<N2,N1,N2> plant = LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1),.00019125,50);
    private DCMotorSim extensionMotorSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);

    private double extensionPosition;
    
    @Override
    public void updateInputs(ExtensionIOInputs inputs) {
        extensionMotorSimulated.update(0.02);

        inputs.extensionSpeedPercent = extensionMotorVoltage/12;
        inputs.extensionAppliedVoltage = extensionMotorVoltage;
        inputs.extensionCurrentAmps = extensionMotorSimulated.getCurrentDrawAmps();
        inputs.extensionPosition += ((extensionMotorSimulated.getAngularVelocityRPM()) / 60 * 0.02) * 4;
        extensionPosition = inputs.extensionPosition;


    }

    @Override
    public void setExtensionPercentOutput(double speed) {
        double speedClamped = speed;
        speedClamped = clampSpeedsExtensionPercent(extensionPosition, speedClamped);
        setExtensionVoltage(speedClamped * 12);
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        extensionMotorVoltage = voltage;
        extensionMotorSimulated.setInputVoltage(voltage);
    }
}
