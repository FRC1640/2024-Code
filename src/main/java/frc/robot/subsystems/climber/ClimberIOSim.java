package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO{
    DCMotorSim leftClimbingMotorSimulated= new DCMotorSim(DCMotor.getNEO(1), 
    50, 0.00019125);
    DCMotorSim rightClimbingMotorSimulated= new DCMotorSim(DCMotor.getNEO(1), 
    50, 0.00019125);
    
    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;

    private double leftMotorPosition = 0.0;
    private double rightMotorPosition = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        leftClimbingMotorSimulated.update(0.02);
        rightClimbingMotorSimulated.update(0.02);

        inputs.leftSpeedPercent = leftMotorVoltage/12;
        inputs.leftAppliedVoltage = leftMotorVoltage;
        inputs.leftCurrentAmps = leftClimbingMotorSimulated.getCurrentDrawAmps();
        inputs.leftClimberPositionDegrees += leftClimbingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        leftMotorPosition = inputs.leftClimberPositionDegrees;

        inputs.rightSpeedPercent = rightMotorVoltage/12;
        inputs.rightAppliedVoltage = rightMotorVoltage;
        inputs.rightCurrentAmps = rightClimbingMotorSimulated.getCurrentDrawAmps();
        inputs.rightClimberPositionDegrees += rightClimbingMotorSimulated.getAngularVelocityRPM() / 60 * 360 * 0.02;
        rightMotorPosition = inputs.rightClimberPositionDegrees;
    }

    @Override
    public void setLeftVoltage(double voltage){
        voltage = MathUtil.clamp(voltage, -12, 12);
        leftClimbingMotorSimulated.setInputVoltage(voltage);
        leftMotorVoltage = voltage;
    }

    @Override
    public void setRightVoltage(double voltage){
        voltage = MathUtil.clamp(voltage, -12, 12);
        rightClimbingMotorSimulated.setInputVoltage(voltage);
        rightMotorVoltage = voltage;
    }

    @Override
    public void setLeftSpeedPercent(double speed){
        speed = clampSpeeds(leftMotorPosition, speed);
        leftClimbingMotorSimulated.setInputVoltage(speed * 12);
    }

    @Override
    public void setRightSpeedPercent(double speed){
        speed = clampSpeeds(rightMotorPosition, speed);
        rightClimbingMotorSimulated.setInputVoltage(speed * 12);

    }

    @Override
    public double getLeftPercentOutput() {
        return leftMotorVoltage / 12;
    }

    @Override
    public double getRightPercentOutput() {
        return rightMotorVoltage / 12;
    }
}
