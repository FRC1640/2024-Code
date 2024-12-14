package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class TargetingIOSim implements TargetingIO {
    LinearSystem<N2,N1,N2> plant = LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .00019125, 50);
    private DCMotorSim leftTargetingMotorSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);
    private DCMotorSim rightTargetingMotorSimulated = new DCMotorSim(plant, DCMotor.getNEO(1), 0);

    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;

    private double leftPositon;
    private double rightPosition;



    public TargetingIOSim() {

    }

    @Override
    public void setTargetingSpeedPercent(double speed) {
        double speedClamped = speed;
        // double averagePosition = getPositionAverage(leftPositon, rightPosition);
        speedClamped = clampSpeeds(rightPosition, speedClamped);
        setTargetingVoltage(speedClamped * 12);

    }

    @Override 
    public double getPositionAverage(double motorOneEncoderValue, double motorTwoEncoderValue) {
        return motorTwoEncoderValue;
    }

    @Override
    public void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // leftMotorVoltage = voltage;
        rightMotorVoltage = voltage;
        double speedClamped = voltage;
        // double averagePosition = getPositionAverage(leftPositon, rightPosition);
        speedClamped = clampSpeeds(rightPosition, speedClamped);
        // leftTargetingMotorSimulated.setInputVoltage(voltage);
        rightTargetingMotorSimulated.setInputVoltage(speedClamped);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {

        leftTargetingMotorSimulated.update(0.02);
        rightTargetingMotorSimulated.update(0.02);

        // inputs.leftTargetingSpeedPercent = leftMotorVoltage/12;
        // inputs.leftTargetingAppliedVoltage = leftMotorVoltage;
        // inputs.leftTargetingCurrentAmps = leftTargetingMotorSimulated.getCurrentDrawAmps();
        // inputs.leftTargetingPositionDegrees += leftTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        // leftPositon = inputs.leftTargetingPositionDegrees;

        inputs.rightTargetingSpeedPercent = rightMotorVoltage/12;
        inputs.rightTargetingAppliedVoltage = rightMotorVoltage;
        inputs.rightTargetingCurrentAmps = rightTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.rightTargetingPositionDegrees += rightTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        rightPosition = inputs.rightTargetingPositionDegrees;

        inputs.targetingPositionAverage = getPositionAverage(leftPositon, rightPosition);
    }
}
