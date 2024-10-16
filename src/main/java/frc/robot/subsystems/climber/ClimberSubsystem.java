package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;
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

public class ClimberSubsystem extends SubsystemBase{
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    ClimberIO io;
    PIDController pidLeft = PIDConstants.constructPID(PIDConstants.climberPID, "climberLeft");
    PIDController pidRight = PIDConstants.constructPID(PIDConstants.climberPID, "climberRight");
    double setpoint = 0;
    private Mechanism2d climberVisualization = new Mechanism2d(5.75, 3);
    private MechanismLigament2d climberLeft = new MechanismLigament2d("leftClimber", 2.5, 0);
    private MechanismLigament2d climberRight = new MechanismLigament2d("rightClimber", 2.5, 0);
    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        MechanismRoot2d leftRoot = climberVisualization.getRoot("leftRoot", 0.25, 0.25);
        MechanismRoot2d rightRoot = climberVisualization.getRoot("rightRoot", 3, 0.25);
        leftRoot.append(climberLeft);
        rightRoot.append(climberRight);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        climberLeft.setAngle(inputs.leftClimberPositionDegrees);
        climberRight.setAngle(inputs.rightClimberPositionDegrees);
        Logger.recordOutput("Climber/ClimberMechanism", climberVisualization);
    }

    // public Command climberPIDCommand(double posLeft, double posRight) {
    //     return setSpeedCommand(()->getPIDSpeed(posLeft, ()->inputs.leftClimberPositionDegrees), 
    //         ()->getPIDSpeed(posRight, ()->inputs.rightClimberPositionDegrees));
    // }

    public Command climberPIDCommandVoltage(DoubleSupplier posLeft, DoubleSupplier posRight) {
        Command c = new Command() {
            @Override
            public void execute() {
                setSpeedVoltage(()->getPIDSpeedLeft(posLeft.getAsDouble()), ()->getPIDSpeedRight(posRight.getAsDouble()));
            }

            @Override
            public void end(boolean interrupted) {
                setSpeedVoltage(()->0,()->0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    private void setSpeedPercent(double leftSpeed, double rightSpeed){
        io.setLeftSpeedPercent(leftSpeed);
        io.setRightSpeedPercent(rightSpeed);
    }

    public void setSpeedVoltage(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        io.setLeftSpeedVoltage(leftSpeed.getAsDouble());
        io.setRightSpeedVoltage(rightSpeed.getAsDouble());
    }

    public Command setSpeedCommand(double leftSpeed, double rightSpeed) {
        Command c = new Command() {
            @Override
            public void execute(){
                setSpeedPercent(leftSpeed, rightSpeed);
            }
            @Override
            public void end(boolean interrupted){
                setSpeedPercent(0, 0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public double getLeftAngle(){
        return inputs.leftClimberPositionDegrees;
    }

    public double getRightAngle(){
        return inputs.rightClimberPositionDegrees;

    }

    public Command setSpeedCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, BooleanSupplier switchClimbers){
            Command c = new Command() {
            @Override
            public void execute(){
                if (!switchClimbers.getAsBoolean()){
                    setSpeedPercent(
                        Math.abs(leftSpeed.getAsDouble()) > 0.1?leftSpeed.getAsDouble():0, 
                        Math.abs(rightSpeed.getAsDouble()) > 0.1?rightSpeed.getAsDouble():0);
                }
                else{
                    setSpeedPercent(
                    Math.abs(rightSpeed.getAsDouble()) > 0.1?rightSpeed.getAsDouble():0,
                    Math.abs(leftSpeed.getAsDouble()) > 0.1?leftSpeed.getAsDouble():0);
                }
            }
            @Override
            public void end(boolean interrupted){
                setSpeedPercent(0, 0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command setSpeedCommandVoltage(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
            Command c = new Command() {
            @Override
            public void execute(){
                setSpeedVoltage(
                    leftSpeed, 
                    rightSpeed);
            }
            @Override
            public void end(boolean interrupted){
                setSpeedPercent(0, 0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    

    public boolean getRightProximitySensor(){
        return inputs.rightProximitySensor;
    }

    public boolean getLeftProximitySensor(){
        return inputs.leftProximitySensor;
    }


    private double getPIDSpeedLeft(double position) {
        double speed = pidLeft.calculate(inputs.leftClimberPositionDegrees, position);
        speed = MathUtil.clamp(speed, -12, 12);
        if (Math.abs(speed) < 0.001) {
            speed = 0;
        }
        return speed;
    }
    private double getPIDSpeedRight(double position) {
        double speed = pidRight.calculate(inputs.rightClimberPositionDegrees, position);
        speed = MathUtil.clamp(speed, -12, 12);
        if (Math.abs(speed) < 0.001) {
            speed = 0;
        }
        return speed;
    }
}
