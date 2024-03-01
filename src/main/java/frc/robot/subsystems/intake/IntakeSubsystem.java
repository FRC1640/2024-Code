package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    IntakeIO io;
    boolean shooting;

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
                    shooting = true;
                }
                else{
                    io.setIntakeSpeedPercent(0);
                    io.setIndexerSpeedPercent(0);
                    shooting = false;
                }
                
            }

            @Override
            public void end(boolean interrupted) {
                io.setIntakeSpeedPercent(0);
                io.setIndexerSpeedPercent(0);
                shooting = false;
            }

            @Override
            public boolean isFinished() {
                return false;
            }

        };
        c.addRequirements(this);
        return c;
    }

    public Command intakeCommand(double speedIntake, double speedIndexer, BooleanSupplier runIntake, double time) {
        Command c = new Command() {

            long initTime;

            @Override
            public void execute() {
                if (runIntake.getAsBoolean() && initTime + time <= System.currentTimeMillis()) {
                    io.setIntakeSpeedPercent(speedIntake);
                    io.setIndexerSpeedPercent(speedIndexer);
                    shooting = true;
                }
                else{
                    io.setIntakeSpeedPercent(0);
                    io.setIndexerSpeedPercent(0);
                    shooting = false;
                }
                
            }

            @Override
            public void end(boolean interrupted) {
                io.setIntakeSpeedPercent(0);
                io.setIndexerSpeedPercent(0);
                shooting = false;
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override 
            public void initialize(){
                initTime = System.currentTimeMillis();
            }

        };
        c.addRequirements(this);
        return c;
    }

    public boolean isShooting(){
        return shooting;
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

}
