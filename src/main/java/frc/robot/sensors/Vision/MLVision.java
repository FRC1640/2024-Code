package frc.robot.sensors.Vision;

import java.util.ArrayList;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.util.Units;
import frc.lib.periodic.PeriodicBase;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.Constants;

public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();
    private double trigDistance;
   
    private LimelightHelpers.LimelightResults  llresults;
    private LimelightHelpers.LimelightTarget_Detector[] resultsArray;
    private LimelightHelpers.LimelightTarget_Detector targetNote;


    public MLVision(MLVisionIO io) {
        this.io = io;

    }

    public void periodic() {
        io.updateInputs(inputs);
        
        llresults = LimelightHelpers.getLatestResults("limelight-ml");
        resultsArray = llresults.targetingResults.targets_Detector;
        targetNote = calculateTargetNote();

        Logger.processInputs("ML Vision", inputs);
        Logger.recordOutput("Distance to note", getDistance());
        io.takeSnapshot(inputs);
    }
    


    // Getters 

    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }

    public double getTX(){   
        return targetNote.tx;
    }
    
    public double getTY(){
        return targetNote.ty;
    }
    
    public double getTA(){
        return targetNote.ta;
    }

    public double getDistance(){
        // Using data points and trigonometry for distance calculations from the
        // Limelight to the object detected.
        if (!inputs.isTarget){
            trigDistance = -1;
        }
        else{
            trigDistance = Units.inchesToMeters( // "d = (h2-h1) / tan(a1+a2)"
                (Constants.LimelightConstants.noteHeightInches - Constants.LimelightConstants.limelightLensHeight)
                        / Math.tan(Math.toRadians(inputs.ty + Constants.LimelightConstants.limelightAngle)));
        
        }
        return trigDistance;
    }

    public LimelightHelpers.LimelightTarget_Detector calculateTargetNote(){
        
        ArrayList<LimelightHelpers.LimelightTarget_Detector> notesArray = new ArrayList<LimelightHelpers.LimelightTarget_Detector>() ;

        double maxTa = -1;        
        int indexWithLargestTa = -1;

        double threshold = 1;

        ArrayList<LimelightHelpers.LimelightTarget_Detector> notesWithinThreshold = new ArrayList<LimelightHelpers.LimelightTarget_Detector>() ;
        double minTX = 27;
        int feasibleIndexWithLeastTX = -1;

        // Get rid of ones with class != note AKA fill notesArray list
        for (int i = 0; i < resultsArray.length; i++) {
            LimelightHelpers.LimelightTarget_Detector detector = resultsArray[i];//.getJSONObject(i);
           
            if (detector.className.equals("note")) { 
                notesArray.add(detector);
            }  
        }

        // Find note with the largest TA
        for (int i = 0; i < notesArray.size(); i++) {
            LimelightHelpers.LimelightTarget_Detector detector = notesArray.get(i);           
             double ta = detector.ta;
           
            if (ta > maxTa) { 
                //update the max with the note currently being read
                maxTa = ta;
                indexWithLargestTa = i;
            }  
        }
        
        //fill feasible notes array (notes within threshold)
        for (LimelightHelpers.LimelightTarget_Detector detector : notesArray) {
            if (detector.ta > maxTa-threshold) { 
                notesWithinThreshold.add(detector);
            }  
        }

        // find the leftmose note within threshold
        
        for (int i = 0; i < notesWithinThreshold.size(); i++) {
            LimelightHelpers.LimelightTarget_Detector detector = notesWithinThreshold.get(i);
            double tx = detector.tx;
            if (tx < minTX) { 
                minTX = tx;
                feasibleIndexWithLeastTX = i;
            }
        }

        return notesWithinThreshold.get(feasibleIndexWithLeastTX);

    }

   
    }
