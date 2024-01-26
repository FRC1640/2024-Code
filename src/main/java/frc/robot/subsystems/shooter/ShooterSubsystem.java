package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    ShooterIO io;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    private void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
    }

    private void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setVoltage(topLeft, bottomLeft, topRight, bottomRight);
    }

    public Command setSpeedCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
            }

            @Override
            public void initialize() {

            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setVoltageCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                setVoltage(topLeft, bottomLeft, topRight, bottomRight);
            }

            @Override
            public void initialize() {

            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }
}
