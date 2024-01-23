package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    IntakeIO io;
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command intakeCommand(double speed) {
        Command c = new Command() {

            @Override
            public void execute() {
                io.setSpeedPercent(speed);
            }

            @Override
            public void end(boolean interrupted) {
                io.setSpeedPercent(0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
            
        };
        c.addRequirements(this);
        return c;
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }
}
