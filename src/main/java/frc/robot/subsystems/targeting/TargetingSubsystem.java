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
        Logger.recordOutput("Targeting/mech", targetVisualization);
    }

    /**
     * Sets the angle of the arm.
     * 
     * @param angle The angle to move to.
     * @return A new RunCommand that sets the speed, setting the speed to 0 if the command is canceled.
     */
    public Command targetFocusPosition(double angle) {
        return setSpeedCommand(()->getPIDSpeed(angle));
    }
    public Command targetFocusPosition(DoubleSupplier angle) {
        return setSpeedCommand(()->getPIDSpeed(angle.getAsDouble()));
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
        return Math.abs(getSetpoint() - inputs.targetingPositionAverage) < error;
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

    /**
     * Returns a Command which sets the motors to a speed,
     *  setting the speed to 0 if the command is canceled.
     * 
     * @param speed the speed to set the motors to.
     * @return New RunCommand.
     */
    public Command setSpeedCommand(double speed) {
        Command c = new Command() {
            @Override
            public void execute(){
                setSpeed(speed);
            }
            @Override
            public void end(boolean interrupted){
                setSpeed(0);
            }
        };
        c.addRequirements(this);
        return c;
    }
    public Command setSpeedCommand(DoubleSupplier speed) {
        Command c = new Command() {
            @Override
            public void execute(){
                setSpeed(speed.getAsDouble());
            }
            @Override
            public void end(boolean interrupted){
                setSpeed(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    /**
     * Returns a Command which sets the motors to a voltage,
     *  setting the voltage to 0 if the command is canceled.
     * 
     * @param voltage the voltage to set the motors to.
     * @return New Command.
     */
    public Command setVoltageCommand(double voltage) {
            Command c = new Command() {
            @Override
            public void execute(){
                setVoltage(voltage);
            }
            @Override
            public void end(boolean interrupted){
                setVoltage(0);
            }
        };
        c.addRequirements(this);
        return c;
    }
}