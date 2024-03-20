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







    // TODO set % speeds in non % methods

    // TODO param names & comments

    // TODO make sure all cancel

    // TODO percent in SV?

    // TODO order in groups

    // TODO conditions in all commands











    // TODO percent


    public Command setSpeedCommand(double speed) {
        return Commands.startEnd(
            () -> setSpeedPercent(speed, speed, speed, speed),
            () -> setSpeedPercent(0, 0, 0, 0),
            this
        );
    }


    public Command setSpeedCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        return Commands.startEnd(
            () -> setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight),
            () -> setSpeedPercent(0, 0, 0, 0),
            this
        );
    }

    public Command setSpeedCommand(DoubleSupplier speed) {
        double speedExtracted = speed.getAsDouble();
        return Commands.startEnd(
            () -> setSpeedPercent(speedExtracted, speedExtracted, speedExtracted, speedExtracted),
            () -> setSpeedPercent(0, 0, 0, 0),
            this
        );
    }


    public Command setSpeedCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight) {

        return Commands.startEnd(
            () -> setSpeedPercent(topLeft.getAsDouble(), bottomLeft.getAsDouble(),
                topRight.getAsDouble(), bottomRight.getAsDouble()),
            () -> setSpeedPercent(0, 0, 0, 0),
            this
        );
    }

    public Command setSpeedCommand(double speed, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSpeedPercent(speed, speed, speed, speed);
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


    public Command setSpeedCommand(double topLeft, double bottomLeft,
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

    public Command setSpeedCommand(DoubleSupplier speed, BooleanSupplier condition) {
        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                  setSpeedPercent(
                    speed.getAsDouble(), speed.getAsDouble(),
                    speed.getAsDouble(), speed.getAsDouble()
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


    public Command setSpeedCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
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


    // TODO voltage


    public Command setVoltageCommand(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        return Commands.startEnd(
            () -> setVoltage(topLeft, bottomLeft, topRight, bottomRight),
            () -> setVoltage(0, 0, 0, 0),
            this
        );
    }

    // public Command setVoltageCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition, double[] speeds) {
    //     Command c = new Command() {
    //         @Override
    //         public void end(boolean interrupted) {
    //             setVoltage(0, 0, 0, 0);
    //         }

    //         @Override
    //         public void execute() {
    //             if (condition.getAsBoolean()){

    //                 setVoltage(topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble());
    //                 targetSpeed = speeds;
    //             }
    //             else{
    //                 setVoltage(0,0,0,0);
    //             }
    //         }

    //         @Override
    //         public void initialize() {

    //         }

    //         @Override
    //         public boolean isFinished() {
    //             return false;
    //         }
    //     };
    //     c.addRequirements(this);
    //     return c;
    // }

    public Command setVoltageCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight) {
        return Commands.startEnd(
            () -> setVoltage(topLeft.getAsDouble(), bottomLeft.getAsDouble(),
                topRight.getAsDouble(), bottomRight.getAsDouble()),
            () -> setVoltage(0, 0, 0, 0),
            this
        );
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.startEnd(
            () -> setVoltage(voltage, voltage, voltage, voltage),
            () -> setVoltage(0, 0, 0, 0),
            this
        );
    }

    public Command setVoltageCommand(DoubleSupplier voltage) {
        return Commands.startEnd(
            () -> setVoltage(voltage.getAsDouble(), voltage.getAsDouble(),
                voltage.getAsDouble(), voltage.getAsDouble()),
            () -> setVoltage(0, 0, 0, 0),
            this
        );
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



    // TODO SV


    // private Command setSmartVelocityCommand(DoubleSupplier topLeft, DoubleSupplier bottomLeft, DoubleSupplier topRight, DoubleSupplier bottomRight, BooleanSupplier condition, double[] speeds) {
    //     Command c = new Command() {
    //         @Override
    //         public void end(boolean interrupted) {
    //             setSmartVelocity(0, 0, 0, 0);
    //         }

    //         @Override
    //         public void execute() {
    //             if (condition.getAsBoolean()){

    //                 setSmartVelocity(topLeft.getAsDouble() * 5480, bottomLeft.getAsDouble() * 5480, topRight.getAsDouble() * 5480, bottomRight.getAsDouble() * 5480);
    //                 targetSpeed = speeds;
    //             }
    //             else{
    //                 setSmartVelocity(0,0,0,0);
    //             }
    //         }

    //         @Override
    //         public void initialize() {

    //         }

    //         @Override
    //         public boolean isFinished() {
    //             return false;
    //         }
    //     };
    //     c.addRequirements(this);
    //     return c;
    // }

    public void setSmartVelocity(double velocity) {
        io.setSmartVelocity(velocity, velocity, velocity, velocity);
    }
    
    public void setSmartVelocity(DoubleSupplier topLeft, DoubleSupplier bottomLeft,
            DoubleSupplier topRight, DoubleSupplier bottomRight) {

        io.setSmartVelocity(
            topLeft.getAsDouble(), bottomLeft.getAsDouble(), topRight.getAsDouble(), bottomRight.getAsDouble()
        );
    }

    public void setSmartVelocity(DoubleSupplier velocity) {
        io.setSmartVelocity(
            velocity.getAsDouble(), velocity.getAsDouble(),
            velocity.getAsDouble(), velocity.getAsDouble()
        );
    }

    public Command setSmartVelocity(double topLeft, double bottomLeft,
            double topRight, double bottomRight, BooleanSupplier condition) {

        Command c = new Command() {
            @Override
            public void execute() {
                if (condition.getAsBoolean()) {
                    setSmartVelocity(topLeft, bottomLeft, topRight, bottomRight);
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




    // TODO good

    











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

    private void setSpeedPercent(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSpeedPercent(topLeft, bottomLeft, topRight, bottomRight);
        targetSpeed = new double[] { topLeft, bottomLeft, topRight, bottomRight };
    }

    private void setVoltage(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setVoltage(topLeft, bottomLeft, topRight, bottomRight);
    }

    private void setSmartVelocity(double topLeft, double bottomLeft, double topRight, double bottomRight) {
        io.setSmartVelocity(topLeft, bottomLeft, topRight, bottomRight);
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
