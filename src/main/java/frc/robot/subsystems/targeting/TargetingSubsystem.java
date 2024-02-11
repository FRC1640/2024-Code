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
    PIDController extensionPID = PIDConstants.constructPID(PIDConstants.extensionPID);
    public double setpoint = 0.0;
    public double extensionSetpoint = 0.0;

    private Mechanism2d targetVisualization = new Mechanism2d(4, 4);
    private MechanismLigament2d angler = new MechanismLigament2d("angler", 1, 0);

    static boolean firstPowerOn = true;
    private DigitalInput limitSwitch = new DigitalInput(TargetingConstants.targetingLimitSwitchId);
    private PIDController encoderMovePID = PIDConstants.constructPID(PIDConstants.limSwitchMovePID);
    private PIDController encoderFreezePID = PIDConstants.constructPID(PIDConstants.limSwitchFreezePID);
    private RunCommand zeroLockCommand;


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
        angler.setLength(Math.max(0.1, inputs.extensionPosition * 1000));
        Logger.recordOutput("Targeting/mech", targetVisualization);
    }

    /**
     * Sets the angle of the arm.
     * 
     * @param angle The angle to move to.
     * @return A new RunCommand that sets the speed, setting the speed to 0 if the command is canceled.
     */
    public Command targetFocusPosition(double angle) {
        releaseZeroLock();
        return new RunCommand(() -> setSpeed(getPIDSpeed(angle)), this)
                .andThen(new InstantCommand(() -> setSpeed(0), this));
    }

    public Command targetFocusPosition(DoubleSupplier angle) {
        releaseZeroLock();
        return new RunCommand(() -> setSpeed(getPIDSpeed(angle.getAsDouble())), this)
                .andThen(new InstantCommand(() -> setSpeed(0), this));
    }

    public Command extendToPosition(double position) {
        return new RunCommand(() -> setExtensionSpeedPercent(getExtensionPIDSpeed(position)), this)
                .andThen(new InstantCommand(() -> setSpeed(0), this));
    }

    /**
     * Calculates the speed to reach the setpoint angle using a PID.
     * 
     * @param position the position to move to.
     * @return The calculated speed.
     */
    private double getPIDSpeed(double position) {
        double speed = pid.calculate(inputs.targetingPositionAverage, position);
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        setpoint = position;
        return speed;
    }

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
     * @return The setpoint.
     */
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * Returns whether the position of the targeting is within the inputted margin of error from the setpoint.
     * 
     * @param error the allowed degree error for the arm.
     * @return Whether the angle is within the margin of error as a boolean.
     */
    public boolean isPositionAccurate(double error) {
        return Math.abs(getSetpoint() - inputs.targetingPositionAverage) < error
        ;
    }

    /**
     * Sets the motor voltage.
     * 
     * @param voltage the voltage to set the motors to.
     */
    private void setVoltage(double voltage) {
        io.setTargetingVoltage(voltage);
    }

    /**
     * Sets the motor speed.
     * 
     * @param speed the speed to set the motors to.
     */
    private void setSpeed(double speed) {
        io.setTargetingSpeedPercent(speed);
    }

    private void setExtensionVoltage(double voltage) {
        io.setExtensionVoltage(voltage);
    }

    private void setExtensionSpeedPercent(double speed) {
        io.setExtensionVoltage(speed);
    }

    /**
     * Returns a RunCommand which sets the motors to a speed,
     *  setting the speed to 0 if the command is canceled.
     * 
     * @param speed the speed to set the motors to.
     * @return New RunCommand.
     */
    public Command setSpeedCommand(double speed) {
        releaseZeroLock();
        return new RunCommand(() -> setSpeed(speed), this)
                .andThen(new InstantCommand(() -> setSpeed(0), this));
    }

    /**
     * Returns a RunCommand which sets the motors to a voltage,
     *  setting the voltage to 0 if the command is canceled.
     * 
     * @param voltage the voltage to set the motors to.
     * @return New RunCommand.
     */
    public Command setVoltageCommand(double voltage) {
        return new RunCommand(() -> setVoltage(voltage), this)
                .andThen(new InstantCommand(() -> setVoltage(0), this));
    }

    /**
     * Sets the speed to move to the limit switch, set the speed to 0,
     * sets the encoder values to 0.
     * 
     * @return RunCommand setting the speed to -0.5.
     */
    public Command resetEncoderCommand() { // TODO speed
        firstPowerOn = false;
        return new RunCommand(() -> setSpeed(
                encoderMovePID.calculate(inputs.targetingPositionAverage, inputs.targetingPositionAverage - 10))) // TODO modification to position
                .until(() -> limitSwitch.get())
                .andThen(new InstantCommand(() -> setSpeed(0)))
                .andThen(new InstantCommand(() -> io.resetEncoderValue()))
                .andThen(zeroLockCommand = new RunCommand(() -> setSpeed(encoderFreezePID.calculate(
                inputs.targetingPositionAverage, 0))));
    }

    /**
     * Releases the lock to zero initialized by <code> resetEncoderCommand() </code>.
     */
    public void releaseZeroLock() {
        if (zeroLockCommand != null){
            zeroLockCommand.cancel();
        } 
    }

    /**
     * Gets whether the motor encoders have been not yet been reset via
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


    public TargetingIO getIO() {
        return io;
    }
}