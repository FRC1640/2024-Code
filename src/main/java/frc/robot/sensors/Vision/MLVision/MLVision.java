package frc.robot.sensors.Vision.MLVision;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.periodic.PeriodicBase;


public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();

    public MLVision(MLVisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("ML Vision", inputs);

    }
    


    // Getters 

    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }

    public double getTX(){
        if (inputs.isTargetNote){   
            return inputs.calculatedTx;
        }
        else{
            return inputs.tx;
        }
    }
    
    public double getTY(){
        if (inputs.isTargetNote){   
            return inputs.calculatedTy;
        }
        else{
            return inputs.ty;
        }
    }
    
    public double getTA(){
        if (inputs.isTargetNote){   
            return inputs.calculatedTa;
        }
        else{
            return inputs.ta;
        }
    }

    public Command waitUntilMLCommand(double taMin, double txMax){
        //return new WaitUntilCommand(()->(getTA() > ta) && (getTX() > tx));
        //return new WaitUntilCommand
        return new WaitUntilCommand(()->isTarget() && getTA() > taMin); 

    }
}