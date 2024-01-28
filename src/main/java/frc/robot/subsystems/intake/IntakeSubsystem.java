package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command intakeCommand(double speedIntake, double speedIndexer, BooleanSupplier runIntake) {
        Command c = new Command() {

            @Override
            public void execute() {
                if (runIntake.getAsBoolean()) {
                    io.setIntakeSpeedPercent(speedIntake);
                    io.setIndexerSpeedPercent(speedIndexer);
                }
            }

            @Override
            public void end(boolean interrupted) {
                io.setIntakeSpeedPercent(0);
                io.setIndexerSpeedPercent(0);
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

    public Command intakeCommand(double speedIntake, double speedIndexer) {
        Command c = new Command() {

            @Override
            public void execute() {
                io.setIntakeSpeedPercent(speedIntake);
                io.setIndexerSpeedPercent(speedIndexer);
            }

            @Override
            public void end(boolean interrupted) {
                io.setIntakeSpeedPercent(0);
                io.setIndexerSpeedPercent(0);
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
