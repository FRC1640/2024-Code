package frc.robot.sensors.Vision.MLVision;

import edu.wpi.first.wpilibj2.command.Command;

public class MLVisionAutonConstraintsCommand extends Command {

    private MLVision vision;
    
    private int TaRange;
    private int TyRange;



    public MLVisionAutonConstraintsCommand(MLVision vision){
        this.vision = vision;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {}

    @Override 
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        if ((vision.getTA() > TaRange) || (vision.getTY() < TyRange) ){
            return false;
        }
        return false;
    }

}
