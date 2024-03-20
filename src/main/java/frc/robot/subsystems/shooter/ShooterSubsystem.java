package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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


    public Command setSpeedPercentPID(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition){
        return setVoltageCommand(
            ()->topLeftPID.calculate(inputs.topLeftVelocity,topLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI), 
            ()->bottomLeftPID.calculate(inputs.bottomLeftVelocity,bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI),
            ()->topRightPID.calculate(inputs.topRightVelocity,topRight.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI),
            ()->bottomRightPID.calculate(inputs.bottomRightVelocity,bottomRight.getAsDouble() * 5676 / 60 * 2 * Math.PI) + ff.calculate(bottomLeft.getAsDouble() * 5676 / 60 * 2 * Math.PI),
            condition, new double[]{topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble()});
    }









    // TODO comments

    public Command setSpeedPercentCommand(double percent) {
        Command c = new Command() {
            @Override
            public void execute() {
                setSpeedPercent(percent, percent, percent, percent);
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        Command c = new Command() {
            @Override
            public void execute() {
                setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(DoubleSupplier percent) {
        Command c = new Command() {
            @Override
            public void execute() {
                setSpeedPercent(percent.getAsDouble(), percent.getAsDouble(), percent.getAsDouble(), percent.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight) {

        Command c = new Command() {
            @Override
            public void execute() {
                setSpeedPercent(
                    topLeft.getAsDouble(), bottomLeft.getAsDouble(),
                    topRight.getAsDouble(), bottomRight.getAsDouble()
                );
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(double percent, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSpeedPercent(percent, percent, percent, percent);
                } else {
                    setSpeedPercent(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(double topLeft, double bottomLeft,
            double topRight, double bottomRight, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
                } else {
                    setSpeedPercent(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(DoubleSupplier percent, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                  setSpeedPercent(
                    percent.getAsDouble(), percent.getAsDouble(),
                    percent.getAsDouble(), percent.getAsDouble()
                  );  
                } else {
                    setSpeedPercent(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedPercentCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSpeedPercent(
                        topLeft.getAsDouble(), bottomLeft.getAsDouble(),
                        topRight.getAsDouble(), bottomRight.getAsDouble()
                    );
                } else {
                    setSpeedPercent(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedPercent(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    private void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
        targetSpeed = new double[] { topLeft, bottomLeft, topRight, bottomRight };
    } // TODO array

    public Command setVoltageCommand(double voltage) {
        Command c = new Command() {
            @Override
            public void execute() {
                setVoltage(voltage, voltage, voltage, voltage);
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
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
            public void execute() {
                setVoltage(topLeft, bottomLeft, topRight, bottomRight);
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setVoltageCommand(DoubleSupplier voltage) {
        Command c = new Command() {
            @Override
            public void execute() {
                setVoltage(
                    voltage.getAsDouble(), voltage.getAsDouble(),
                    voltage.getAsDouble(), voltage.getAsDouble()
                );
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setVoltageCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight) {
        Command c = new Command() {
            @Override
            public void execute() {
                setVoltage(
                    topLeft.getAsDouble(), bottomLeft.getAsDouble(),
                    topRight.getAsDouble(), bottomRight.getAsDouble()
                );
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setVoltageCommand(double voltage, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setVoltage(voltage, voltage, voltage, voltage);
                } else {
                    setVoltage(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }    

    public Command setVoltageCommand(double topLeft, double bottomLeft,
            double topRight, double bottomRight, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setVoltage(topLeft, bottomLeft, topRight, bottomRight);
                } else {
                    setVoltage(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setVoltageCommand(DoubleSupplier voltage, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setVoltage(
                        voltage.getAsDouble(), voltage.getAsDouble(),
                        voltage.getAsDouble(), voltage.getAsDouble()
                    );
                } else {
                    setVoltage(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setVoltageCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition) {
        
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setVoltage(
                        topLeft.getAsDouble(), bottomLeft.getAsDouble(),
                        topRight.getAsDouble(), bottomRight.getAsDouble()
                    );
                } else {
                    setVoltage(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setVoltage(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    private void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setVoltage(topLeft, bottomLeft, topRight, bottomRight);
        targetSpeed = new double[] { topLeft / 12, bottomLeft / 12, topRight / 12, bottomRight / 12 };
    }

    public Command setSmartVelocityPercentCommand(double percent) {

        Command c = new Command() {
            @Override
            public void execute() {
                setSmartVelocity(percent * 5480, percent * 5480, percent * 5480, percent * 5480);
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(double topLeft, double bottomLeft,
            double topRight, double bottomRight) {

        Command c = new Command() {
            @Override
            public void execute() {
                setSmartVelocity(topLeft * 5480, bottomLeft * 5480, topRight * 5480, bottomRight * 5480);
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(DoubleSupplier percent) {

        Command c = new Command() {
            @Override
            public void execute() {
                setSmartVelocity(
                    percent.getAsDouble() * 5480, percent.getAsDouble() * 5480,
                    percent.getAsDouble() * 5480, percent.getAsDouble() * 5480
                );
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight) {

        Command c = new Command() {
            @Override
            public void execute() {
                setSmartVelocity(
                    topLeft.getAsDouble() * 5480, bottomLeft.getAsDouble() * 5480,
                    topRight.getAsDouble() * 5480, bottomRight.getAsDouble() * 5480
                );
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(double percent, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSmartVelocity(percent * 5480, percent * 5480, percent * 5480, percent * 5480);
                } else {
                    setSmartVelocity(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(double topLeft, double bottomLeft,
            double topRight, double bottomRight, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSmartVelocity(topLeft * 5480, bottomLeft * 5480, topRight * 5480, bottomRight * 5480);
                } else {
                    setSmartVelocity(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(DoubleSupplier percent, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSmartVelocity(
                        percent.getAsDouble() * 5480, percent.getAsDouble() * 5480,
                        percent.getAsDouble() * 5480, percent.getAsDouble() * 5480
                    );
                } else {
                    setSmartVelocity(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSmartVelocityPercentCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSmartVelocity(
                        topLeft.getAsDouble() * 5480, bottomLeft.getAsDouble() * 5480,
                        topRight.getAsDouble() * 5480, bottomRight.getAsDouble() * 5480
                    );
                } else {
                    setSmartVelocity(0, 0, 0, 0);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setSmartVelocity(0, 0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    private void setSmartVelocity(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSmartVelocity(topLeft, bottomLeft, topRight, bottomRight);
        targetSpeed = new double[] { topLeft / 5480, bottomLeft / 5480, topRight / 5480, bottomRight / 5480 };
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

    public double getSpeedRadians(){
        return inputs.topLeftVelocity;
    }

    public double getPosRadians(){
        return inputs.topLeftPositionRadians;
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
}
