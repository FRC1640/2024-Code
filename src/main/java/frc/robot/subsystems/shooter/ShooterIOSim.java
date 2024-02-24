package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

    private DCMotorSim topLeftShooterSimulated =
        new DCMotorSim(DCMotor.getNEO(1), 10, 0.00019125);
    private DCMotorSim bottomLeftShooterSimulated =
        new DCMotorSim(DCMotor.getNEO(1), 10, 0.00019125);
    private DCMotorSim topRightShooterSimulated =
        new DCMotorSim(DCMotor.getNEO(1), 10, 0.00019125);
    private DCMotorSim bottomRightShooterSimulated =
        new DCMotorSim(DCMotor.getNEO(1), 10, 0.00019125);

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

    @Override
    public void testTopLeftSpeed(double speed) {
        MathUtil.clamp(speed, -1, 1);
        topLeftShooterSimulated.setInputVoltage(speed * 12);
        topLeftVoltage = speed * 12;
    }

    @Override
    public void testTopRightSpeed(double speed) {
        MathUtil.clamp(speed, -1, 1);
        topRightShooterSimulated.setInputVoltage(speed * 12);
        topRightVoltage = speed * 12;
    }

    @Override
    public void testBottomLeftSpeed(double speed) {
        MathUtil.clamp(speed, -1, 1);
        bottomLeftShooterSimulated.setInputVoltage(speed * 12);
        bottomLeftVoltage = speed * 12;
    }

    @Override
    public void testBottomRightSpeed(double speed) {
        MathUtil.clamp(speed, -1, 1);
        bottomRightShooterSimulated.setInputVoltage(speed * 12);
        bottomRightVoltage = speed * 12;
    }

    @Override
    public double getTopLeftSpeed() {
        return topLeftVoltage / 12;
    }

    @Override
    public double getTopRightSpeed() {
        return topRightVoltage / 12;
    }

    @Override
    public double getBottomLeftSpeed() {
        return bottomLeftVoltage / 12;
    }

    @Override
    public double getBottomRightSpeed() {
        return bottomRightVoltage / 12;
    }
}
