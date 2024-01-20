package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    ShooterIO io;

    public ShooterSubsystem(ShooterIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    private void setSpeedPercent(double top, double bottom){
        io.setSpeedPercent(top, bottom);
    }

    private void setVoltage(double top, double bottom){
        io.setVoltage(top,bottom);
    }

    public Command setSpeedCommand(double topSpeed, double bottomSpeed){
        Command c = new Command(){
            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0);
            }

            @Override
            public void execute() {
                setSpeedPercent(topSpeed, bottomSpeed);
            }

            @Override
            public void initialize() {
            
            }

            @Override
            public boolean isFinished() {
                return false;
            }};
            c.addRequirements(this);
            return c;
    }

    public Command setVoltageCommand(double topVolt, double bottomVolt){
        Command c =  new Command(){
            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0);
            }

            @Override
            public void execute() {
              setVoltage(topVolt, bottomVolt);
            }

            @Override
            public void initialize() {
            
            }

            @Override
            public boolean isFinished() {
                return false;
            }};
            c.addRequirements(this);
            return c;
    }
}
