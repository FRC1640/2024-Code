package frc.robot.subsystems.targeting;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;

public class TargetingSubsystem extends SubsystemBase {
    TargetingIOInputsAutoLogged inputs = new TargetingIOInputsAutoLogged();
    TargetingIO io;
    PIDController pid = PIDConstants.constructPID(PIDConstants.targetingPID);
    public double setpoint = 0.0;

    private Mechanism2d targetVisualization = new Mechanism2d(4, 4);
    private MechanismLigament2d angler = new MechanismLigament2d("angler", 1, 0);

    public double extensionSetpoint = 0.0;
    PIDController extensionPID = PIDConstants.constructPID(PIDConstants.extensionPID);

    

    public TargetingSubsystem(TargetingIO io) {
        this.io = io;
        MechanismRoot2d root = targetVisualization.getRoot("targeter", 2, 2);
        root.append(angler);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Targeting", inputs);
        angler.setAngle(inputs.targetingPositionAverage);
        angler.setLength(inputs.extensionPosition + 0.5);
        Logger.recordOutput("Targeting/mech", targetVisualization);
    }

    /**
     * Combines <code> anglePIDCommand() </code> and <code> extensionPIDCommand() </code>,
     * running both methods with the inputted angle and position.
     * 
     * @param angle The angle to move to.
     * @param position The position to extend to.
     * @return Command[] of angle and extension commands.
     */
    public Command[] extendAndAngle(double angle, double position) {
        return new Command[] {anglePIDCommand(angle), extensionPIDCommand(position)};
    }

    /**
     * Combines <code> anglePIDCommand() </code> and <code> extensionPIDCommand() </code>,
     * running both methods with the inputted angle supplier and position.
     * 
     * @param angle Supplier giving the angle to move to.
     * @param position The position to extend to.
     * @return Command[] of angle and extension commands.
     */
    public Command[] extendAndAngle(DoubleSupplier angle, double position) {
        return new Command[] {anglePIDCommand(angle), extensionPIDCommand(position)};
    }

    /**
     * Combines <code> anglePIDCommand() </code> and <code> extensionPIDCommand() </code>,
     * running both methods with the inputted position supplier and angle.
     * 
     * @param angle The angle to move to.
     * @param position Supplier giving the position to extend to.
     * @return Command[] of angle and extension commands.
     */
    public Command[] extendAndAngle(double angle, DoubleSupplier position) {
        return new Command[] {anglePIDCommand(angle), extensionPIDCommand(position)};
    }

    /**
     * Combines <code> anglePIDCommand() </code> and <code> extensionPIDCommand() </code>,
     * running both methods with the inputted angle and position suppliers.
     * 
     * @param angle Supplier giving the angle to move to.
     * @param position Supplier giving the position to extend to.
     * @return Command[] of angle and extension commands.
     */
    public Command[] extendAndAngle(DoubleSupplier angle, DoubleSupplier position) {
        return new Command[] {anglePIDCommand(angle), extensionPIDCommand(position)};
    }

    /**
     * Moves to the given angle.
     * 
     * @param angle The angle to move to.
     * @return New Command.
     */
    public Command anglePIDCommand(double angle) {
        return setAnglePercentOutputCommand(() -> getAnglePIDSpeed(angle));
    }

    /**
     * Moves to the given angle.
     * 
     * @param angle Supplier giving the angle to move to.
     * @return New Command.
     */
    public Command anglePIDCommand(DoubleSupplier angle) {
        return setAnglePercentOutputCommand(() -> getAnglePIDSpeed(angle.getAsDouble()));
    }

    /**
     * Calculates the percent output of the angle motors needed
     * to reach the inputted angle as quickly as possible.
     * 
     * @param position The angle to move to.
     * @return Percent output.
     */
    private double getAnglePIDSpeed(double position) {
        double speed = pid.calculate(inputs.targetingPositionAverage, position);
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        setpoint = position;
        return speed;
    }

    /**
     * Returns a new Command which sets the angler to a percent output,
     * setting the percent output to 0 when the command ends.
     * 
     * @param output The percent output to set the angle motors to.
     * @return New Command.
     */
    public Command setAnglePercentOutputCommand(double output) {
        Command c = new Command() {
            @Override
            public void execute(){
                setAnglePercentOutput(output);
            }
            @Override
            public void end(boolean interrupted){
                setAnglePercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Returns a new Command which sets the angler to a percent output,
     * setting the percent output to 0 when the command ends.
     * 
     * @param output Supplier giving the percent output to set the angle motors to.
     * @return New Command.
     */
    public Command setAnglePercentOutputCommand(DoubleSupplier output) {
        Command c = new Command() {
            @Override
            public void execute(){
                setAnglePercentOutput(output.getAsDouble());
            }
            @Override
            public void end(boolean interrupted){
                setAnglePercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Returns a new Command which sets the angler to a voltage,
     * setting the voltage to 0 when the command ends.
     * 
     * @param voltage The voltage to set the angle motors to.
     * @return New Command.
     */
    public Command setAngleVoltageCommand(double voltage) {
            Command c = new Command() {
            @Override
            public void execute(){
                setAngleVoltage(voltage);
            }
            @Override
            public void end(boolean interrupted){
                setAngleVoltage(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Sets the percent output of the angle motors.
     * 
     * @param output The percent output to set the motors to.
     */
    private void setAnglePercentOutput(double output) {
        io.setTargetingSpeedPercent(output);
    }

    /**
     * Sets the voltage of the angle motors.
     * 
     * @param voltage The voltage to set the motors to.
     */
    private void setAngleVoltage(double voltage) {
        io.setTargetingVoltage(voltage);
    }

    /**
     * Gets the setpoint of the angler.
     * 
     * @return The setpoint.
     */
    public double getAngleSetpoint() {
        return setpoint;
    }

    /**
     * Returns whether the position of the angler is within the inputted margin of error from the setpoint.
     * 
     * @param error The allowed error in degrees for the arm.
     * @return Whether the angle versus the setpoint is within the margin of error as a boolean.
     */
    public boolean isAnglePositionAccurate(double error) {
        return Math.abs(getAngleSetpoint() - inputs.targetingPositionAverage) < error;
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
                setExtensionPercentOutput(getExtensionPIDSpeed(position));
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
                setExtensionPercentOutput(getExtensionPIDSpeed(position.getAsDouble()));
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
     * Calculates the percent output of the extension motor needed
     * to reach the inputted position as quickly as possible.
     * 
     * @param position The position to extend to.
     * @return Percent output.
     */
    private double getExtensionPIDSpeed(double position) {
        double speed = extensionPID.calculate(inputs.extensionPosition, position);
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        return speed;
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
     * @param output Supplier giving the percent output to set the extension motor to.
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

    /**
     * Gets the position of the extension.
     * 
     * @return The position of the extension.
     */
    public double getExtensionPosition() {
        return getIO().getExtensionPosition();
    }

    /**
     * Returns the TargetingIO passed into the subsystem's constructor.
     * 
     * @return The subsystem's TargetingIO.
     */
    private TargetingIO getIO() {
        return io;
    }
}