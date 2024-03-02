package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import frc.lib.periodic.PeriodicBase;
import frc.lib.vision.LimelightHelpers;

public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();   
    private LimelightHelpers.LimelightResults  llresults;
    private LimelightHelpers.LimelightTarget_Detector[] resultsArray;

    private LimelightHelpers.LimelightTarget_Detector targetNote;
    private boolean isTargetNote = false; // has the target note been set

    private int numNotesInView = 0;
    private int numNotesWithinThreshold = 0;



    public MLVision(MLVisionIO io) {
        this.io = io;
        llresults = LimelightHelpers.getLatestResults("limelight-ml");
        resultsArray = llresults.targetingResults.targets_Detector;
    }

    public void periodic() {
        io.updateInputs(inputs);
        
        llresults = LimelightHelpers.getLatestResults("limelight-ml");
        resultsArray = llresults.targetingResults.targets_Detector;
        numNotesInView = resultsArray.length;

        if (isTarget()){
            // targetNote = calculateTargetNote();
            isTargetNote = true;
        }
        else{
            isTargetNote = false;
        }

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
        if (isTargetNote){   
            return targetNote.tx;
        }
        else{
            return inputs.tx;
        }
    }
    
    public double getTY(){
        if (isTargetNote){   
            return targetNote.ty;
        }
        else{
            return inputs.ty;
        }
    }
    
    public double getTA(){
        if (isTargetNote){   
            return targetNote.ta;
        }
        else{
            return inputs.ta;
        }
    }
}