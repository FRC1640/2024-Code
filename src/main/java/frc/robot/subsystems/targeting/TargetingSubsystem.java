package frc.robot.subsystems.targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.sysid.ArmSysidRoutine;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TargetingConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

public class TargetingSubsystem extends SubsystemBase {
    TargetingIOInputsAutoLogged inputs = new TargetingIOInputsAutoLogged();
    TargetingIO io;
    PIDController pidSmall = PIDConstants.constructPID(PIDConstants.targetingPIDSmall, "targetingPIDSmall");
    PIDController pidSuperSmall = PIDConstants.constructPID(PIDConstants.targetingPIDSuperSmall, "targetingPIDSuperSmall");
    PIDController pidLarge = PIDConstants.constructPID(PIDConstants.targetingPIDLarge, "targetingPIDLarge");
    public double setpoint = 0.0;

    private Mechanism2d targetVisualization = new Mechanism2d(4, 4);
    private MechanismLigament2d angler = new MechanismLigament2d("angler", 1, 0);

    SysIdRoutine sysIdRoutine;

    ArmFeedforward feedforward = new ArmFeedforward(0.55, 0.08, 0.75);
    PIDController ffPID = new PIDController(0.01, 0, 0);

    PIDController radianAngle = PIDConstants.constructPID(PIDConstants.radianAngle, "radian angle");
    private DoubleSupplier angleOffset;

    public TargetingSubsystem(TargetingIO io, DoubleSupplier angleOffset) {
        this.io = io;
        this.angleOffset = angleOffset;
        MechanismRoot2d root = targetVisualization.getRoot("targeter", 2, 2);
        root.append(angler);

        sysIdRoutine = new ArmSysidRoutine().createNewRoutine(
                this::setAngleVoltage, this::getAngleVoltage, this::getAnglePosition,
                this::getAngleVelocity, this, new SysIdRoutine.Config(mutable(Volts.of(0.05)).per(Seconds.of(1)),
                        mutable(Volts.of(0.75)), mutable(Second.of(60))));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Targeting", inputs);
        angler.setAngle(inputs.targetingPosition);
        // angler.setLength(inputs.extensionPosition / 40 * 2 * Math.PI );
        Logger.recordOutput("Targeting/mech", targetVisualization);
        // Logger.recordOutput("Targeting/velocity", inputs.);

        Logger.recordOutput("Targeting/angleoffset", angleOffset.getAsDouble());

    }

    // public Command manualFeedForwardAngle(DoubleSupplier speed){
    // return
    // setAngleVoltageCommand(feedforward.calculate(Math.toRadians(inputs.rightTargetingPositionDegrees),
    // speed.getAsDouble())
    // + ffPID.calculate(inputs.rightRadiansPerSecond, speed.getAsDouble()));
    // }
    // private Command pidFeedForwardAngle(DoubleSupplier speed){
    // return
    // setAngleVoltageCommand(feedforward.calculate(Math.toRadians(inputs.rightTargetingPositionDegrees),
    // speed.getAsDouble()) + ffPID.calculate(inputs.rightRadiansPerSecond,
    // speed.getAsDouble()));
    // }

    public Command pidFeedForwardCommand(DoubleSupplier pos) {
        return setAngleVoltageCommand(() -> feedforward.calculate(Math.toRadians(inputs.rightTargetingPositionDegrees),
                getRadianPIDSpeed(() -> pos.getAsDouble()))
                + ffPID.calculate(inputs.rightRadiansPerSecond, getRadianPIDSpeed(() -> pos.getAsDouble())));
    }

    private double getRadianPIDSpeed(DoubleSupplier pos) {

        double speed = radianAngle.calculate(Math.toRadians(inputs.rightTargetingPositionDegrees), pos.getAsDouble());
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        setpoint = Math.toDegrees(pos.getAsDouble());
        return speed;
    }

    public double distToAngle(DoubleSupplier dist) {
        Logger.recordOutput("AutoTargetAngle", dist.getAsDouble() < 2.4 ? 60 : equation(dist.getAsDouble()));
        return dist.getAsDouble() < 2.4 ? 60 : equation(dist.getAsDouble());
    }

    public double equation(double v) {

        return -2530.15 * Math.asin(-0.000478453 * (1 / (v-0.0742577))-0.999783) -3910.04 + angleOffset.getAsDouble() + 2.5 + 1;
    }

    public double getAngleVoltage() {
        return inputs.rightTargetingAppliedVoltage;
    }

    public double getAnglePosition() {
        return Math.toRadians(inputs.rightTargetingPositionDegrees);
    }

    public double getAngleVelocity() {
        return inputs.rightRadiansPerSecond;
    }

    /**
     * Moves to the given angle.
     * 
     * @param angle The angle to move to.
     * @return New Command.
     */
    public Command anglePIDCommand(double angle) {
        return setAngleVoltageCommand(() -> getAnglePIDSpeed(angle));
    }

    public Command anglePIDCommand(DoubleSupplier angle, double limit) {
        return setAngleVoltageCommand(() -> getAnglePIDSpeed(angle.getAsDouble()), limit);
    }

    public Command anglePIDCommand(DoubleSupplier angle, double limit, BooleanSupplier condition) {
        return setAngleVoltageCommand(() -> getAnglePIDSpeed(angle.getAsDouble()), limit, condition);
    }

    public Command anglePIDCommand(DoubleSupplier angle) {
        return setAngleVoltageCommand(() -> getAnglePIDSpeed(angle.getAsDouble()));
    }

    public Command anglePIDCommand(DoubleSupplier angle, BooleanSupplier condition) {
        return setAngleVoltageCommand(() -> getAnglePIDSpeed(angle.getAsDouble()), condition);
    }

    /**
     * Calculates the percent output of the angle motors needed
     * to reach the inputted angle as quickly as possible.
     * 
     * @param targetPosition The angle to move to.
     * @return Percent output.
     */
    private double getAnglePIDSpeed(double targetPosition) {
        if (targetPosition <= TargetingConstants.angleLowerLimit) {
            pidSmall.reset();
        }
        double speed = 0;
        if (Math.abs(targetPosition - inputs.targetingPosition) < 1){
            speed = pidSuperSmall.calculate(inputs.targetingPosition, targetPosition);
        }
        else if (Math.abs(targetPosition - inputs.targetingPosition) < 6){
            speed = pidSmall.calculate(inputs.targetingPosition, targetPosition);
        }
        else{
            speed = pidLarge.calculate(inputs.targetingPosition, targetPosition);
        }
        speed = MathUtil.clamp(speed, -12, 12);
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        setpoint = targetPosition;
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
            public void execute() {
                setAnglePercentOutput(output);
            }

            @Override
            public void end(boolean interrupted) {
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
            public void execute() {
                setAnglePercentOutput(output.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                setAnglePercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setAnglePercentOutputCommand(DoubleSupplier output, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setAnglePercentOutput(output.getAsDouble());
                } else {
                    setAnglePercentOutput(0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setAnglePercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setAnglePercentOutputCommand(DoubleSupplier output, double limit) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (getAnglePosition() > limit) {
                    setAnglePercentOutput(Math.min(output.getAsDouble(), 0));
                } else {
                    setAnglePercentOutput(output.getAsDouble());
                }
            }

            @Override
            public void end(boolean interrupted) {
                setAnglePercentOutput(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setAnglePercentOutputCommand(DoubleSupplier output, double limit, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    if (getAnglePosition() > limit) {
                        setAnglePercentOutput(Math.min(output.getAsDouble(), 0));
                    } else {
                        setAnglePercentOutput(output.getAsDouble());
                    }
                } else {
                    setAnglePercentOutput(0);
                }
            }

            @Override
            public void end(boolean interrupted) {
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
    public Command setAngleVoltageCommand(DoubleSupplier voltage, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setAngleVoltage(voltage.getAsDouble());
                } else {
                    setAngleVoltage(0);
                }

            }

            @Override
            public void end(boolean interrupted) {
                setAngleVoltage(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setAngleVoltageCommand(DoubleSupplier voltage, double limit) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (getAnglePosition() > limit) {
                    setAngleVoltage(Math.min(voltage.getAsDouble(), 0));
                } else {
                    setAngleVoltage(voltage.getAsDouble());
                }

            }

            @Override
            public void end(boolean interrupted) {
                setAngleVoltage(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setAngleVoltageCommand(DoubleSupplier voltage, double limit, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    if (getAnglePosition() > limit) {
                        setAngleVoltage(Math.min(voltage.getAsDouble(), 0));
                    } else {
                        setAngleVoltage(voltage.getAsDouble());
                    }
                }
                else{
                    setAngleVoltage(0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setAngleVoltage(0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setAngleVoltageCommand(DoubleSupplier voltage) {
        Command c = new Command() {
            @Override
            public void execute() {
                setAngleVoltage(voltage.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
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
     * Returns whether the position of the angler is within the inputted margin of
     * error from the setpoint.
     * 
     * @param error The allowed error in degrees for the arm.
     * @return Whether the angle versus the setpoint is within the margin of error
     *         as a boolean.
     */
    public boolean isAnglePositionAccurate(double error) {
        return Math.abs(getAngleSetpoint() - inputs.targetingPosition) < error;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void runBlower(double speed){
        io.runBlower(speed);
    }

    public Command runBlowerCommand(double speed){
        Command c = new Command() {
            @Override
            public void execute(){
                runBlower(speed);
            }
            @Override
            public void end(boolean interrupted){
                runBlower(0);
            }
        };
        return c;
    }


    //testing thingies
    public Command upCommand(){
        Command c = new Command () {
            @Override
            public void execute(){
                setpoint = 65;
            }
        };
        return c;
    }

    public Command downCommand(){
        Command c = new Command () {
            @Override
            public void execute(){
                setpoint = 35;
            }
        };
        return c;
    }

    public Command midCommand(){
        Command c = new Command () {
            @Override
            public void execute(){
                setpoint = 50;
            }
        };
        return c;
    }
    
}