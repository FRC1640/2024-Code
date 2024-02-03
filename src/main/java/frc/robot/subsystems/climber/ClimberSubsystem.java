package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    ClimberIO io;
    
    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs", inputs);
    }

    private void setSpeedPercent(double speed){
        io.setLeftSpeedPercent(speed);
        io.setRightSpeedPercent(speed);
    }

    public Command runClimberCommand(double percentage){
        return Commands.run(()->setSpeedPercent(percentage))
            .finallyDo(() -> setSpeedPercent(0));
    }
}
