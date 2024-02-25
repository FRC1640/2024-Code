package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.sysid.ArmSysidRoutine;
import frc.robot.Constants.PIDConstants;
import frc.robot.util.dashboard.PIDUpdate;

public class ShooterSubsystem extends SubsystemBase {
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    ShooterIO io;
    double[] targetSpeed = new double[] { 0, 0, 0, 0 };

    SysIdRoutine sysIdRoutine;

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(2, 0.01);

    private PIDController topLeftPID = PIDConstants.constructPID(PIDConstants.shooterVelocityPID, "topLeftShooter");
    private PIDController bottomLeftPID = PIDConstants.constructPID(PIDConstants.shooterVelocityPID, "bottomLeftShooter");
    private PIDController topRightPID = PIDConstants.constructPID(PIDConstants.shooterVelocityPID, "topRightShooter");
    private PIDController bottomRightPID = PIDConstants.constructPID(PIDConstants.shooterVelocityPID, "bottomRightShooter");
    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        topLeftPID.setIntegratorRange(-3, 3);
        bottomLeftPID.setIntegratorRange(-3, 3);
        topRightPID.setIntegratorRange(-3, 3);
        bottomRightPID.setIntegratorRange(-3, 3);
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

        Logger.recordOutput("pid", bottomLeftPID.getSetpoint());

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

    public Command setSpeedPercentPID(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition){
        return setVoltageCommand(
            ()->topLeftPID.calculate(inputs.topLeftVelocity,topLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI), 
            ()->bottomLeftPID.calculate(inputs.bottomLeftVelocity,bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI),
            ()->topRightPID.calculate(inputs.topRightVelocity,topRight.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI),
            ()->bottomRightPID.calculate(inputs.bottomRightVelocity,bottomRight.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI),
            condition, new double[]{topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble()});
    }

    private void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setVoltage(topLeft, bottomLeft, topRight, bottomRight);
    }

    public Command setSpeedCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
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

    public Command setSpeedCommand(double speed) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                setSpeedPercent(speed, speed, speed, speed);
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

    public Command setVoltageCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        Command c = new Command() {
            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public void execute() {
                setVoltage(topLeft, bottomLeft, topRight, bottomRight);
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

                    // System.out.println("good: " + speeds[0].getAsDouble());
                    setVoltage(topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble());
                    targetSpeed = speeds;
                }
                else{
                    // System.out.println("AHHHHHH");
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
