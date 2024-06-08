package frc.robot.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ExtensionIOSim implements ExtensionIO{

    private double extensionMotorVoltage = 0.0;
    private DCMotorSim extensionMotorSimulated = new DCMotorSim(DCMotor.getNEO(1),
        50, 0.00019125);

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
