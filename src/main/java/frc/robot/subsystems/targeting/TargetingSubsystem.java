package frc.robot.subsystems.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;

public class TargetingSubsystem extends SubsystemBase {
    TargetingIOInputsAutoLogged inputs = new TargetingIOInputsAutoLogged();
    TargetingIO io;
    PIDController pid = PIDConstants.constructPID(PIDConstants.targetingPID);
    public double setpoint = 0.0;

    public TargetingSubsystem(TargetingIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /**
     * Sets the angle of the arm.
     * 
     * @param angle The angle to move to.
     * @return A new RunCommand that sets the speed, setting the speed to 0 if the command is canceled.
     */
    public Command targetFocusPosition(double angle) {
        return new RunCommand(() -> setSpeed(getPIDSpeed(angle)), this)
                .andThen(new InstantCommand(() -> setSpeed(0), this));
    }

    private double getPIDSpeed(double position) {
        double speed = pid.calculate(inputs.targetingPositionAverage, position);
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.1) {
            speed = 0;
        }
        setpoint = position;
        return speed;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean isPositionAccurate(double error) {
        return Math.abs(getSetpoint() - inputs.targetingPositionAverage) < error;
    }

    private void setVoltage(double voltage) {
        io.setTargetingVoltage(voltage);
    }

    private void setSpeed(double speed) {
        io.setTargetingSpeedPercent(speed);
    }

    public Command setSpeedCommand(double speed) {
        return new RunCommand(() -> setSpeed(speed), this)
                .andThen(new InstantCommand(() -> setSpeed(0), this));
    }

    public Command setVoltageCommand(double voltage) {
        return new RunCommand(() -> setVoltage(voltage), this)
                .andThen(new InstantCommand(() -> setVoltage(0), this));
    }
}