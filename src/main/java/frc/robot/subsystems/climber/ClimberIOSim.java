package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO{
    DCMotorSim leftClimbingMotorSimulated= new DCMotorSim(DCMotor.getNEO(1), 
    50, 0.00019125);
    DCMotorSim rightClimbingMotorSimulated= new DCMotorSim(DCMotor.getNEO(1), 
    50, 0.00019125);
    
    private double leftMotorVoltage = 0.0;
    private double rightMotorVoltage = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        leftClimbingMotorSimulated.update(0.02);
        rightClimbingMotorSimulated.update(0.02);

        inputs.leftSpeedPercent = leftMotorVoltage/12;
        inputs.leftAppliedVoltage = leftMotorVoltage;
        inputs.leftCurrentAmps = leftClimbingMotorSimulated.getCurrentDrawAmps();

        inputs.rightSpeedPercent = rightMotorVoltage/12;
        inputs.rightAppliedVoltage = rightMotorVoltage;
        inputs.rightCurrentAmps = rightClimbingMotorSimulated.getCurrentDrawAmps();
    }

    @Override
    public void setLeftVoltage(double voltage){
        leftClimbingMotorSimulated.setInputVoltage(voltage);
        leftMotorVoltage = voltage;
    }

    @Override
    public void setRightVoltage(double voltage){
        rightClimbingMotorSimulated.setInputVoltage(voltage);
        rightMotorVoltage = voltage;
    }

    @Override
    public void setLeftSpeedPercent(double speed){
        leftClimbingMotorSimulated.setInput(speed);
        leftMotorVoltage = speed * 12;
    }

    @Override
    public void setRightSpeedPercent(double speed){
        rightClimbingMotorSimulated.setInput(speed);
        rightMotorVoltage = speed * 12;
    }
}
