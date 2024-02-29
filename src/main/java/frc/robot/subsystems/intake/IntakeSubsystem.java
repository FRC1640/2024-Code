package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    IntakeIO io;

    private Command testCommand;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command intakeCommand(double speedIntake, double speedIndexer, BooleanSupplier runIntake) {
        Command c = new Command() {

            @Override
            public void execute() {
                if (runIntake.getAsBoolean()) {
                    io.setIntakeSpeedPercent(speedIntake);
                    io.setIndexerSpeedPercent(speedIndexer);
                }
                else{
                    io.setIntakeSpeedPercent(0);
                    io.setIndexerSpeedPercent(0);
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
        // System.out.println(inputs.hasNote);
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

    public Command intakeNoteCommand(double speedIntake, double speedIndexer, BooleanSupplier hasNote){
        Command c = new Command() {

            @Override
            public void execute() {
                if (!hasNote.getAsBoolean()){
                    io.setIntakeSpeedPercent(speedIntake);
                    io.setIndexerSpeedPercent(speedIndexer);
                }
                else{
                    io.setIntakeSpeedPercent(0);
                    io.setIndexerSpeedPercent(0);
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

    public Command testIntakeSpeedCommand(DoubleSupplier intakeSpeed) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                io.setIntakeSpeedPercent(intakeSpeed.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testIndexerSpeedCommand(DoubleSupplier indexerSpeed) {
        Command c = new Command() {
            @Override
            public void initialize() {
                testCommand = this;
            }
            @Override
            public void execute() {
                io.setIndexerSpeedPercent(indexerSpeed.getAsDouble());
            }
            @Override
            public boolean isFinished() {
                return testCommand != this;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public double getIntakePercentOutput() {
        return io.getIntakePercentOutput();        
    }

    public double getIndexerPercentOutput() {
        return io.getIndexerPercentOutput();        
    }

    // TODO BAD METHODS! GET RID OF THEM!

    public void testIntakeSpeed(double speed) {
        io.setIntakeSpeedPercent(speed);
    }

    public void testIndexerSpeed(double speed) {
        io.setIndexerSpeedPercent(speed);
    }
}
