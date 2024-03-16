package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.sysid.ArmSysidRoutine;

public class ShooterSubsystem extends SubsystemBase {
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    ShooterIO io;
    double[] targetSpeed = new double[] { 0, 0, 0, 0 };

    SysIdRoutine sysIdRoutine;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        sysIdRoutine = new ArmSysidRoutine().createNewRoutine(
            this::setVoltageFL, this::getVoltageFL, this::getPosRadians, 
            this::getSpeedRadians, this, new SysIdRoutine.Config());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/TargetSpeeds", getTargetSpeeds());
        Logger.recordOutput("Shooter/ActualSpeeds", getSpeeds());

        // PIDUpdate.update(bottomLeftPID);
    }

    public void setVoltageFL(double v){
        io.setVoltage(v, 0, 0, 0);
    }
    public double getVoltageFL(){
        return inputs.topLeftAppliedVoltage;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
        targetSpeed = new double[] { topLeft, bottomLeft, topRight, bottomRight };
    }

    private void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setVoltage(topLeft, bottomLeft, topRight, bottomRight);
    }

    public Command setSpeedCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        return Commands.startEnd(
            () -> setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight),
            () -> setSpeedPercent(0, 0, 0, 0),
            this
        );
    }

    public Command setSpeedPercentCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition) {

        return setSmartVelocityCommand(
            topLeft, bottomRight, topRight, bottomRight, condition,
            new double[] {
                topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble()
            }
        );

    }

    private Command setSmartVelocityCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition, double[] speeds) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                if (condition.getAsBoolean()){

                    setSmartVelocity(topLeft.getAsDouble() * 5480, bottomLeft.getAsDouble() * 5480, topRight.getAsDouble() * 5480, bottomRight.getAsDouble() * 5480);
                    targetSpeed = speeds;
                }
                else{
                    setSmartVelocity(0,0,0,0);
                }
            }

            @Override
            public void initialize() {

            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public void setSmartVelocity(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSmartVelocity(topLeft, bottomLeft, topRight, bottomRight);
    }

    public Command setSpeedCommand(double speed) {
        return Commands.startEnd(
            () -> setSpeedPercent(speed, speed, speed, speed),
            () -> setSpeedPercent(0, 0, 0, 0),
            this
        );
    }

    public Command setVoltageCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        return Commands.startEnd(
            () -> setVoltage(topLeft, bottomLeft, topRight, bottomRight),
            () -> setVoltage(0, 0, 0, 0),
            this
        );
    }

    public double getSpeedRadians(){
        return inputs.topLeftVelocity;
    }

    public double getPosRadians(){
        return inputs.topLeftPositionRadians;
    }

    public Command setVoltageCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition, double[] speeds) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                if (condition.getAsBoolean()){

                    setVoltage(topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble());
                    targetSpeed = speeds;
                }
                else{
                    setVoltage(0,0,0,0);
                }
            }

            @Override
            public void initialize() {

            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public double[] getSpeeds() {
        return new double[] { inputs.topLeftSpeedPercent, inputs.bottomLeftSpeedPercent, inputs.topRightSpeedPercent,
                inputs.bottomRightSpeedPercent };
    }

    public double[] getTargetSpeeds() {
        return targetSpeed;
    }

    public boolean isSpeedAccurate(double percentError) {
        int count = 0;
        for (double d : getTargetSpeeds()) {
            if (Math.abs(d - getSpeeds()[count]) > percentError) {
                Logger.recordOutput("Shooter/error", Math.abs(d - getSpeeds()[count]));
                return false;
            }
            count += 1;
        }
     
        return true;
    }
}
