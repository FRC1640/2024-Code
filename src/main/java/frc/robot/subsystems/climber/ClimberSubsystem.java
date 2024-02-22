package frc.robot.subsystems.climber;

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
    PIDController pid = PIDConstants.constructPID(PIDConstants.climberPID, "climber");
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
        Logger.processInputs("Inputs", inputs);
        climberLeft.setAngle(inputs.leftClimberPositionDegrees);
        climberRight.setAngle(inputs.rightClimberPositionDegrees);
        Logger.recordOutput("Climber/ClimberMechanism", climberVisualization);
    }

    public Command climberPIDCommand(double posLeft, double posRight) {
        return setSpeedCommand(()->getPIDSpeed(posLeft, ()->inputs.leftClimberPositionDegrees), 
            ()->getPIDSpeed(posRight, ()->inputs.rightClimberPositionDegrees));
    }

    private void setSpeedPercent(double leftSpeed, double rightSpeed){
        io.setLeftSpeedPercent(leftSpeed);
        io.setRightSpeedPercent(rightSpeed);
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

    public Command setSpeedCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
            Command c = new Command() {
            @Override
            public void execute(){
                setSpeedPercent(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
            }
            @Override
            public void end(boolean interrupted){
                setSpeedPercent(0, 0);
            }
        };
        c.addRequirements(this);
        return c;
    }

    private double getPIDSpeed(double position, DoubleSupplier getPos) {
        double speed = pid.calculate(getPos.getAsDouble(), position);
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }
        setpoint = position;
        return speed;
    }

    public double getPercentOutput() {
        return (io.getLeftPercentOutput() + io.getRightPercentOutput()) / 2;
    }
}
