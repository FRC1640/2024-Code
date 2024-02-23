package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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

    public Command testIntakeSpeedCommand(double intakeSpeed) {
        Command c = new Command() {
            @Override
            public void execute() {
                io.setIntakeSpeedPercent(intakeSpeed);
            }
            @Override
            public void end(boolean interrupted) {
                io.setIntakeSpeedPercent(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command testIndexerSpeedCommand(double indexerSpeed) {
        Command c = new Command() {
            @Override
            public void execute() {
                io.setIndexerSpeedPercent(indexerSpeed);
            }
            @Override
            public void end(boolean interrupted) {
                io.setIndexerSpeedPercent(0);
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

    public DCMotorSim getSimulationIntakeMotorTest() {
        return io.getSimulationIntakeMotor();
    }

    public CANSparkMax getRealIntakeMotorTest() {
        return io.getRealIntakeMotor();
    }

    public DCMotorSim getSimulationIndexerMotorTest() {
        return io.getSimulationIntakeMotor();
    }

    public CANSparkMax getRealIndexerMotorTest() {
        return io.getRealIntakeMotor();
    }
}
