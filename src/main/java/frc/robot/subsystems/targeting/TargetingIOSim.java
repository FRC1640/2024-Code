package frc.robot.subsystems.targeting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class TargetingIOSim implements TargetingIO {
    private DCMotorSim leftTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);
    private DCMotorSim rightTargetingMotorSimulated = new DCMotorSim(DCMotor.getNEO(1), 
        50, 0.00019125);

    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;

    private double leftPositon;
    private double rightPosition;



    public TargetingIOSim() {

    }

    @Override
    public void setTargetingSpeedPercent(double speed) {  // TODO negative or positive limits & speeds
        double speedClamped = speed;
        double averagePosition = getPositionAverage(leftPositon, rightPosition);
        // if (averagePosition < TargetingConstants.targetingLowerLimit) {
        //     speedClamped = Math.max(speed, 0);
        // }
        // if (averagePosition > TargetingConstants.targetingUpperLimit) {
        //     speedClamped = Math.min(speed, 0);
        // }
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
        inputs.leftTargetingPositionDegrees += leftTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        leftPositon = inputs.leftTargetingPositionDegrees;

        inputs.rightTargetingSpeedPercent = rightMotorVoltage/12;
        inputs.rightTargetingAppliedVoltage = rightMotorVoltage;
        inputs.rightTargetingCurrentAmps = rightTargetingMotorSimulated.getCurrentDrawAmps();
        inputs.rightTargetingPositionDegrees += rightTargetingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        rightPosition = inputs.rightTargetingPositionDegrees;

        inputs.targetingPositionAverage = getPositionAverage(leftPositon, rightPosition);
    }

    public double encoderToDegrees(double motorEncoderValue) { // TODO conversion
        return motorEncoderValue;
    }
}
