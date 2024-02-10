package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    ClimberIO io;
    
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

    private void setSpeedPercent(double speed){
        io.setLeftSpeedPercent(speed);
        io.setRightSpeedPercent(speed);
    }

    // public Command runClimberCommand(double percentage){
    //     return Commands.run(()->setSpeedPercent(percentage))
    //         .finallyDo(() -> setSpeedPercent(0));
    //}
//might change this to two different commands later idk
    public Command runClimberRight(double rSpeed){
        Command c = new Command(){
            @Override
            public void initialize(){

            }

            @Override
            public void execute(){
                io.setRightSpeedPercent(rSpeed);
                // if(0 < inputs.rightClimberPositionDegrees && inputs.rightClimberPositionDegrees < 90){
                // io.setRightSpeedPercent(rSpeed);
                // }
                // else if(inputs.rightClimberPositionDegrees > 90){
                //     io.setRightSpeedPercent(rSpeed * .3);
                // }
                // else {
                //     io.setRightSpeedPercent(0);
                // }
            }

            @Override 
            public void end(boolean interrupted){
                io.setRightSpeedPercent(0);
            }

            @Override
            public boolean isFinished(){
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    public Command runClimberLeft(double lSpeed){
        Command c = new Command(){
            @Override
            public void initialize(){

            }

            @Override
            public void execute(){
                io.setLeftSpeedPercent(lSpeed);
                // if(0 < inputs.leftClimberPositionDegrees && inputs.leftClimberPositionDegrees < 90){
                // io.setLeftSpeedPercent(lSpeed);
                // }
                // else if(inputs.leftClimberPositionDegrees > 90){
                //     io.setLeftSpeedPercent(lSpeed * .3);
                // }
                // else {
                //     io.setLeftSpeedPercent(0);
                // }
            }

            @Override 
            public void end(boolean interrupted){
                io.setLeftSpeedPercent(0);
            }

            @Override
            public boolean isFinished(){
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }
}
