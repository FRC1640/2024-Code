package frc.robot.subsystems.extension;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TargetingConstants;
public class ExtensionSubsystem extends SubsystemBase{
    ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
    ExtensionIO io;
    public double extensionSetpoint = 0.0;
    // PIDController pid = PIDConstants.constructPID(PIDConstants.targetingPID, "angle");

    PIDController extensionPID = PIDConstants.constructPID(PIDConstants.extensionPID, "extension");
    public ExtensionSubsystem(ExtensionIO io){
        this.io = io;
        if (inputs.extensionLimitSwitch){
            io.resetExtension();
        }
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Extension", inputs);
    }

        /**
     * Calculates the percent output of the extension motor needed
     * to reach the inputted position as quickly as possible.
     * 
     * @param position The position to extend to.
     * @return Percent output.
     */
    private double getExtensionPIDSpeed(double position) {
        double speed = extensionPID.calculate(inputs.extensionPosition, position);
        speed = MathUtil.clamp(speed, -12, 12);
        if (Math.abs(speed) < 0.001) {
            speed = 0;
        }
        return speed;
    }
    public boolean isExtensionAccurate(){
        return Math.abs(inputs.extensionPosition - 100) < 6;
    }

        /**
     * Extends to the given position.
     * 
     * @param position The position to extend to.
     * @return New Command.
     */
    public Command extensionPIDCommand(double position) {
        Command c = new Command() {
            @Override
            public void execute() {
                setExtensionVoltage(getExtensionPIDSpeed(position));
            }

            @Override
            public void end(boolean interrupted) {
                setExtensionPercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Extends to the given position.
     * 
     * @param position Supplier giving the position to extend to.
     * @return New Command.
     */
    public Command extensionPIDCommand(DoubleSupplier position) {
        Command c = new Command() {
            @Override
            public void execute() {
                setExtensionVoltage(()->getExtensionPIDSpeed(position.getAsDouble()));
            }

            @Override
            public void end(boolean interrupted) {
                setExtensionPercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Returns a new Command which sets the extension to a percent output,
     * setting the percent output to 0 when the command ends.
     * 
     * @param output The percent output to set the extension motor to.
     * @return New Command.
     */
    public Command setExtensionPercentOutputCommand(double output) {
        Command c = new Command() {
            @Override
            public void execute() {
                setExtensionPercentOutput(output);
            }

            @Override
            public void end(boolean interrupted) {
                setExtensionPercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Returns a new Command which sets the extension to a percent output,
     * setting the percent output to 0 when the command ends.
     * 
     * @param output Supplier giving the percent output to set the extension motor
     *               to.
     * @return New Command.
     */
    public Command setExtensionPercentOutputCommand(DoubleSupplier output) {
        Command c = new Command() {
            @Override
            public void execute() {
                setExtensionPercentOutput(output.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                setExtensionPercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Returns a new Command which sets the extension to a voltage,
     * setting the voltage to 0 when the command is canceled.
     * 
     * @param voltage The voltage to set the extension motor to.
     * @return New Command.
     */
    public Command setExtensionVoltageCommand(double voltage) {
        Command c = new Command() {
            @Override
            public void execute() {
                setExtensionVoltage(voltage);
            }

            @Override
            public void end(boolean interrupted) {
                setExtensionVoltage(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Sets the percent output of the extension motor.
     * 
     * @param output The percent output to set the motor to.
     */
    private void setExtensionPercentOutput(double output) {
        io.setExtensionPercentOutput(output);
    }

    /**
     * Sets the voltage of the extension motor.
     * 
     * @param voltage The voltage to set the motor to.
     */
    private void setExtensionVoltage(double voltage) {
        io.setExtensionVoltage(voltage);
    }

    private void setExtensionVoltage(DoubleSupplier voltage) {
        io.setExtensionVoltage(voltage.getAsDouble());
    }

    

    /**
     * Gets the position of the extension.
     * 
     * @return The position of the extension.
     */
    public double getExtensionPosition() {
        return inputs.extensionPosition;
    }


}
