package frc.robot.subsystems.targeting;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TargetingConstants;



public class TargetingSubsystem extends SubsystemBase {

    TargetingIOInputsAutoLogged inputs = new TargetingIOInputsAutoLogged();
    TargetingIO io;

    PIDController pid = PIDConstants.constructPID(PIDConstants.targetingPID);
    public double setpoint = 0.0;
    
    public double extensionSetpoint = 0.0;
    PIDController extensionPID = PIDConstants.constructPID(PIDConstants.extensionPID);

    private Mechanism2d targetVisualization = new Mechanism2d(4, 4);
    private MechanismLigament2d angler = new MechanismLigament2d("angler", 1, 0);

    static boolean firstPowerOn = true;
    private DigitalInput limitSwitch = new DigitalInput(TargetingConstants.targetingLimitSwitchId);
    private PIDController encoderMovePID = PIDConstants.constructPID(PIDConstants.limSwitchMovePID);


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
     * Combines <code> target() </code> and <code> extend() </code>,
     * running both methods with the inputted angle and position.
     * 
     * @param angle The angle to target.
     * @param position The position to extend to.
     * @return Command[] of targeting RunCommand and extension RunCommand.
     */
    public Command[] extendAndTarget(double angle, double position) {
        return new Command[] {target(angle), extend(position)};
    }

    /**
     * Combines <code> target() </code> and <code> extend() </code>,
     * running both methods with the inputted angle supplier and position.
     * 
     * @param angle Supplier giving the angle to target.
     * @param position The position to extend to.
     * @return Command[] of targeting RunCommand and extension RunCommand.
     */
    public Command[] extendAndTarget(DoubleSupplier angle, double position) {
        return new Command[] {target(angle), extend(position)};
    }

    /**
     * Combines <code> target() </code> and <code> extend() </code>,
     * running both methods with the inputted position supplier and angle.
     * 
     * @param angle The angle to target.
     * @param position Supplier giving the position to extend to.
     * @return Command[] of targeting RunCommand and extension RunCommand.
     */
    public Command[] extendAndTarget(double angle, DoubleSupplier position) {
        return new Command[] {target(angle), extend(position)};
    }

    /**
     * Combines <code> target() </code> and <code> extend() </code>,
     * running both methods with the inputted angle and position suppliers.
     * 
     * @param angle Supplier giving the angle to target.
     * @param position Supplier giving the position to extend to.
     * @return Command[] of targeting RunCommand and extension RunCommand.
     */
    public Command[] extendAndTarget(DoubleSupplier angle, DoubleSupplier position) {
        return new Command[] {target(angle), extend(position)};
    }

    /**
     * Targets the given angle.
     * 
     * @param angle The angle to target.
     * @return New RunCommand.
     */
    public Command target(double angle) {
        return new RunCommand(() -> setTargetingPercentOutput(getTargetingPIDSpeed(angle)), this)
                .andThen(new InstantCommand(() -> setTargetingPercentOutput(0), this));
    }

    /**
     * Targets the given angle.
     * 
     * @param angle Supplier giving the angle to target.
     * @return New RunCommand.
     */
    public Command target(DoubleSupplier angle) {
        return new RunCommand(() -> setTargetingPercentOutput(getTargetingPIDSpeed(angle.getAsDouble())), this)
                .andThen(new InstantCommand(() -> setTargetingPercentOutput(0), this));
    }

    /**
     * Extends to the given position.
     * 
     * @param position The position to extend to.
     * @return New RunCommand.
     */
    public Command extend(double position) {
        return new RunCommand(() -> setTargetingPercentOutput(getExtensionPIDSpeed(position)), this)
                .andThen(new InstantCommand(() -> setTargetingPercentOutput(0), this));
    }

    /**
     * Extends to the given position.
     * 
     * @param position Supplier giving the position to extend to.
     * @return New RunCommand.
     */
    public Command extend(DoubleSupplier position) {
        return new RunCommand(() -> setTargetingPercentOutput(getExtensionPIDSpeed(position.getAsDouble())), this)
                .andThen(new InstantCommand(() -> setTargetingPercentOutput(0), this));
    }

    /**
     * Calculates the percent output of the targeting motor needed
     * to reach the inputted angle as quickly as possible.
     * 
     * @param position The angle to target.
     * @return Percent output.
     */
    private double getTargetingPIDSpeed(double position) {
        double speed = pid.calculate(inputs.targetingPositionAverage, position);
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        setpoint = position;
        return speed;
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
     * Gets the setpoint of the targeting.
     * 
     * @return the setpoint.
     */
    public double getSetAngle() {
        return setpoint;
    }

    /**
     * Returns whether the position of the targeting is within the inputted margin of error from the setpoint.
     * 
     * @param error the allowed error in degrees for the arm.
     * @return Whether the angle versus the setpoint is within the margin of error as a boolean.
     */
    public boolean isAngleAccurate(double error) {
        return Math.abs(getSetAngle() - inputs.targetingPositionAverage) < error;
    }

    /**
     * Sets the voltage of the targeting motors.
     * 
     * @param voltage the voltage to set the motors to.
     */
    private void setTargetingVoltage(double voltage) {
        io.setTargetingVoltage(voltage);
    }

    /**
     * Sets the percent output of the targeting motors.
     * 
     * @param output the percent output to set the motors to.
     */
    private void setTargetingPercentOutput(double output) {
        io.setTargetingPercentOutput(output);
    }

    /**
     * Sets the voltage of the extension motor.
     * 
     * @param voltage the voltage to set the motor to.
     */
    private void setExtensionVoltage(double voltage) {
        io.setExtensionVoltage(voltage);
    }

    /**
     * Sets the percent output of the extension motor.
     * 
     * @param output the percent output to set the motor to.
     */
    private void setExtensionPercentOutput(double output) {
        io.setExtensionPercentOutput(output);
    }

    /**
     * Returns a RunCommand which sets the extension to a percent output,
     * setting the percent output to 0 if the command is canceled.
     * 
     * @param output the percent output to set the extension motor to.
     * @return New RunCommand.
     */
    public Command setExtensionOutputCommand(double output) {
        return new RunCommand(() -> setExtensionPercentOutput(output), this);
    }

    /**
     * Returns a RunCommand which sets the extension to a voltage,
     * setting the voltage to 0 if the command is canceled.
     * 
     * @param voltage the voltage to set the extension motor to.
     * @return new RunCommand.
     */
    public Command setExtensionVoltageCommand(double voltage) {
        return new RunCommand(() -> setExtensionVoltage(voltage), this)
                .andThen(new InstantCommand(() -> setExtensionVoltage(0), this));
    }

    /**
     * Returns a RunCommand which sets the targeting to a percent output,
     * setting the percent output to 0 if the command is canceled.
     * 
     * @param output the percent output to set the targeting motors to.
     * @return new RunCommand.
     */
    public Command setTargetingOutputCommand(double output) {
        return new RunCommand(() -> setTargetingPercentOutput(output), this)
                .andThen(new InstantCommand(() -> setTargetingPercentOutput(0), this));
    }

    /**
     * Returns a RunCommand which sets the targeting to a voltage,
     * setting the voltage to 0 if the command is canceled.
     * 
     * @param voltage the voltage to set the targeting motors to.
     * @return new RunCommand.
     */
    public Command setTargetingVoltageCommand(double voltage) {
        return new RunCommand(() -> setTargetingVoltage(voltage), this)
                .andThen(new InstantCommand(() -> setTargetingVoltage(0), this));
    }

    /**
     * Sets the targeting percent output to move to the reset limit switch, sets the output to 0,
     * and finally sets the targeting encoder values to 0.
     * 
     * @return RunCommand setting the percent output to -0.5.
     */
    public Command resetEncoderCommand() { // TODO speed & in comments
        firstPowerOn = false;
        return new RunCommand(() -> setTargetingPercentOutput(
                encoderMovePID.calculate(inputs.targetingPositionAverage, inputs.targetingPositionAverage - 10))) // TODO modification to position
                .until(() -> limitSwitch.get())
                .andThen(new InstantCommand(() -> setTargetingPercentOutput(0)))
                .andThen(new InstantCommand(() -> io.resetEncoderValue()));
    }

    /**
     * Gets whether the targeting motor encoders have been not yet been reset via
     * <code> resetEncoderCommand() </code>.
     * 
     * <p>
     * Note that this method will return <em> true </em> when
     * <code> resetEncoderCommand() </code>
     * has <em> not </em> yet run.
     * 
     * @return Whether <code> resetEncoderCommand() </code> has NOT yet been run as
     *         a boolean.
     */
    public static boolean getIfEncodersNotReset() {
        return firstPowerOn;
    }

    /**
     * Returns the TargetingIO passed into the subsystem's constructor.
     * 
     * @return The subsystem's TargetingIO.
     */
    public TargetingIO getIO() {
        return io;
    }
}