package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ShooterIOSim implements ShooterIO {
    LinearSystem<N2,N1,N2> plant = LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .00019125, 10);
    private DCMotorSim topLeftShooterSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);
    private DCMotorSim bottomLeftShooterSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);
    private DCMotorSim topRightShooterSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);
    private DCMotorSim bottomRightShooterSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);

    private double topLeftVoltage = 0.0;
    private double bottomLeftVoltage = 0.0;
    private double topRightVoltage = 0.0;
    private double bottomRightVoltage = 0.0;

    @Override
    public void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        setVoltage(topLeft * 12, bottomLeft * 12, topRight * 12, bottomRight * 12);
    }

    @Override
    public void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        topLeft = MathUtil.clamp(topLeft, -12, 12);
        bottomLeft = MathUtil.clamp(bottomLeft, -12, 12);
        topRight = MathUtil.clamp(topRight, -12, 12);
        bottomRight = MathUtil.clamp(bottomRight, -12, 12);
        topLeftShooterSimulated.setInputVoltage(topLeft);
        bottomLeftShooterSimulated.setInputVoltage(bottomLeft);
        topRightShooterSimulated.setInputVoltage(topRight);
        bottomRightShooterSimulated.setInputVoltage(bottomRight);
        topLeftVoltage = topLeft;
        bottomLeftVoltage = bottomLeft;
        topRightVoltage = topRight;
        bottomRightVoltage = bottomRight;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topLeftSpeedPercent = topLeftVoltage/12;
        inputs.topLeftAppliedVoltage = topLeftVoltage;
        inputs.topLeftCurrentAmps = topLeftShooterSimulated.getCurrentDrawAmps();

        inputs.bottomLeftSpeedPercent = bottomLeftVoltage/12;
        inputs.bottomLeftAppliedVoltage = bottomLeftVoltage;
        inputs.bottomLeftCurrentAmps = bottomLeftShooterSimulated.getCurrentDrawAmps();

        inputs.topRightSpeedPercent = topRightVoltage/12;
        inputs.topRightAppliedVoltage = topRightVoltage;
        inputs.topRightCurrentAmps = topRightShooterSimulated.getCurrentDrawAmps();

        inputs.bottomRightSpeedPercent = bottomRightVoltage/12;
        inputs.bottomRightAppliedVoltage = bottomRightVoltage;
        inputs.bottomRightCurrentAmps = bottomRightShooterSimulated.getCurrentDrawAmps();
    }
}
