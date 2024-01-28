package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TargetingConstants;

public class TargetingIOSim implements TargetingIO {
    private DCMotorSim leftTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        0.05, 0.00019125);
    private DCMotorSim rightTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        0.05, 0.00019125);

    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;

    public TargetingIOSim() {
    }

    @Override
    public void setTargetingSpeedPercent(double speed) {  // TODO negative or positive limits & speeds
        double speedClamped = 0;
        double averagePosition = getPositionAverage(leftTargetingMotorSimulated.getAngularPositionRad(),
            rightTargetingMotorSimulated.getAngularPositionRad());
        averagePosition = Math.toDegrees(averagePosition);
        if (averagePosition < TargetingConstants.targetingLowerLimit) {
            speedClamped = Math.max(speed, 0);
        }
        if (averagePosition > TargetingConstants.targetingUpperLimit) {
            speedClamped = Math.min(speed, 0);
        }
        setTargetingVoltage(speedClamped * 12);

    }

    @Override
    public void setTargetingVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        leftMotorVoltage = voltage;
        rightMotorVoltage = voltage;
        leftTargetingMotorSimulated.setInputVoltage(voltage);
        rightTargetingMotorSimulated.setInputVoltage(voltage);
    }

    @Override
    public void updateInputs(TargetingIOInputs inputs) {

        leftTargetingMotorSimulated.update(0.02);
        rightTargetingMotorSimulated.update(0.02);

        inputs.leftTargetingSpeedPercent = leftMotorVoltage/12;
        inputs.leftTargetingAppliedVoltage = leftMotorVoltage;
        inputs.leftTargetingCurrentAmps = leftTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.leftTargetingPositionDegrees = Math.toDegrees(leftTargetingMotorSimulated.getAngularPositionRad());

        inputs.rightTargetingSpeedPercent = rightMotorVoltage/12;
        inputs.rightTargetingAppliedVoltage = rightMotorVoltage;
        inputs.rightTargetingCurrentAmps = rightTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.rightTargetingPositionDegrees = Math.toDegrees(rightTargetingMotorSimulated.getAngularPositionRad());

        inputs.targetingPositionAverage = getPositionAverage(Math.toDegrees(leftTargetingMotorSimulated.getAngularPositionRad()),
                Math.toDegrees(rightTargetingMotorSimulated.getAngularPositionRad()));
    }

    public double encoderToDegrees(double motorEncoderValue) { // TODO conversion
        return motorEncoderValue;
    }
}
